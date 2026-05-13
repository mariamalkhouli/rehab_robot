# =============================================================================
#  therapy/session_manager.py
#  /home/rehabrobot/rehab_robot/therapy/session_manager.py
#
#  Session Manager — Coordinates the therapy session lifecycle.
#
#  RESPONSIBILITIES:
#    1. Creates and ends session records in the database
#    2. Manages which therapy mode is currently running
#    3. Owns the sensor data logging buffer (flushes to DB periodically)
#    4. Tracks repetition count
#    5. Handles E-stop mid-session (marks session as 'estopped' in DB)
#    6. Provides the active session state to Flask routes (api.py)
#
#  DESIGN RULES:
#    - Only ONE session can be active at a time
#    - SessionManager does NOT directly command motors — each mode does that
#    - SessionManager does NOT read sensors — SensorHub does that
#    - SessionManager coordinates: starts modes, stops them, logs data
#    - All database writes are batched (CFG.database.session_log_batch_size)
#
#  SESSION STATES:
#    IDLE       — no session, ready to start
#    STARTING   — session record created, mode being launched
#    RUNNING    — mode actively commanding motors
#    PAUSED     — mode suspended, motors holding position
#    STOPPING   — mode shutting down, flushing logs
#    ENDED      — finished normally
#    ESTOPPED   — terminated by E-stop
#
#  USAGE from api.py:
#    mgr.start_session(patient_id, mode, leg, hip_flex_max, knee_flex_max,
#                      ab_ad_max, speed_deg_per_sec, reps_target)
#    mgr.pause_session()
#    mgr.resume_session()
#    mgr.stop_session()
#    mgr.handle_estop()
#    mgr.update_rom(hip_flex_max, knee_flex_max, speed, ...)
#    mgr.get_status()
#
#  USAGE from therapy modes:
#    mgr.increment_reps()
#    mgr.is_reps_target_reached()
#    mgr.should_continue()
#    mgr.is_paused()
#    mgr.get_rom()
#    mgr.log_sample(angles, fsr_raw, emg_raw, emg_rms,
#                   patient_effort_pct, robot_effort_pct, intent_direction)
# =============================================================================

import threading
import time
import logging
from enum import Enum, auto
from typing import Optional, List

from core.config import CFG
from core.database import (
    create_session,
    end_session,
    bulk_log_samples,
    log_estop_event,
    log_system_event,
)

logger = logging.getLogger(__name__)


# =============================================================================
#  Session State Enum
# =============================================================================

class SessionState(Enum):
    IDLE      = auto()
    STARTING  = auto()
    RUNNING   = auto()
    PAUSED    = auto()
    STOPPING  = auto()
    ENDED     = auto()
    ESTOPPED  = auto()


# =============================================================================
#  SessionManager
# =============================================================================

class SessionManager:
    """
    Coordinates the full lifecycle of a therapy session.

    Instantiated once in main.py and injected into:
      - web/routes/api.py   — HTTP session control endpoints
      - web/routes/ws.py    — WebSocket status streaming
      - therapy/passive_mode.py
      - therapy/active_assist.py
      - therapy/mimicry_mode.py
    """

    def __init__(self, serial_comm, sensor_hub):
        """
        Args:
            serial_comm : core.serial_comm.SerialComm instance (None = dev mode)
            sensor_hub  : core.sensor_hub.SensorHub instance  (None = dev mode)
        """
        self._serial  = serial_comm
        self._sensor  = sensor_hub
        self._lock    = threading.RLock()   # Reentrant — safe for nested calls

        # ── Session state ──────────────────────────────────────────────────
        self._state         : SessionState  = SessionState.IDLE
        self._session_id    : Optional[int] = None
        self._patient_id    : Optional[int] = None
        self._mode          : Optional[str] = None
        self._leg           : str           = 'right'
        self._start_time    : Optional[float] = None

        # ── ROM / motion parameters ────────────────────────────────────────
        # These are updated live by update_rom() and read every cycle
        # by therapy modes via get_rom().
        self._hip_flex_max  : float = 90.0
        self._knee_flex_max : float = 90.0
        self._ab_ad_max     : float = 25.0
        self._speed         : float = CFG.passive_mode.default_speed_deg_per_sec
        self._reps_target   : int   = 10
        self._reps_done     : int   = 0
        self._pain_score    : int   = 0

        # ── Active mode object ─────────────────────────────────────────────
        # Holds reference to PassiveMode / ActiveAssist / MimicryMode instance
        self._mode_obj = None

        # ── Log buffer ─────────────────────────────────────────────────────
        # Samples are appended here by log_sample() (called at 20Hz from modes)
        # and flushed to SQLite by _flush_loop() every flush_interval seconds.
        self._log_buffer  : List[dict] = []
        self._log_lock    = threading.Lock()
        self._flush_interval = CFG.database.session_log_flush_interval_sec
        self._batch_size     = CFG.database.session_log_batch_size

        # ── Flush thread ───────────────────────────────────────────────────
        self._flush_stop   = threading.Event()
        self._flush_thread = threading.Thread(
            target = self._flush_loop,
            name   = "SessionMgr-Flush",
            daemon = True,
        )
        self._flush_thread.start()

        logger.info("SessionManager initialised.")

    # =========================================================================
    #  Public — called from api.py (HTTP endpoints)
    # =========================================================================

    def start_session(
        self,
        patient_id       : int,
        mode             : str,
        leg              : str   = 'right',
        hip_flex_max     : float = 90.0,
        knee_flex_max    : float = 90.0,
        ab_ad_max        : float = 25.0,
        speed_deg_per_sec: float = None,
        reps_target      : int   = 10,
    ) -> dict:
        """
        Start a new therapy session.

        Creates the session record in SQLite, stores ROM parameters,
        and launches the therapy mode thread.

        Args:
            patient_id        : Patient DB primary key
            mode              : 'passive' | 'active' | 'mimicry'
            leg               : 'right' | 'left'
            hip_flex_max      : Maximum hip flexion target (degrees)
            knee_flex_max     : Maximum knee flexion target (degrees)
            ab_ad_max         : Maximum hip ab/adduction target (degrees)
            speed_deg_per_sec : CPM angular velocity. None = config default.
            reps_target       : Target repetition count

        Returns:
            {'ok': True,  'session_id': int, 'state': str}
            {'ok': False, 'error': str}
        """
        with self._lock:
            if self._state not in (
                SessionState.IDLE, SessionState.ENDED, SessionState.ESTOPPED
            ):
                return {
                    'ok'    : False,
                    'error' : f"Cannot start — current state is {self._state.name}",
                }

            if mode not in ('passive', 'active', 'mimicry'):
                return {'ok': False, 'error': f"Unknown mode: {mode}"}

            # Store ROM parameters
            self._patient_id    = patient_id
            self._mode          = mode
            self._leg           = leg
            self._hip_flex_max  = float(hip_flex_max)
            self._knee_flex_max = float(knee_flex_max)
            self._ab_ad_max     = float(ab_ad_max)
            self._speed = float(speed_deg_per_sec) if speed_deg_per_sec is not None \
                          else CFG.passive_mode.default_speed_deg_per_sec
            self._reps_target   = int(reps_target)
            self._reps_done     = 0
            self._pain_score    = 0
            self._start_time    = time.time()

            # Create session record in database
            try:
                self._session_id = create_session(
                    patient_id        = patient_id,
                    therapist_id      = None,
                    mode              = mode,
                    hip_ab_max    = self._ab_ad_max,  
                    hip_flex_max  = self._hip_flex_max,   
                    knee_flex_max = self._knee_flex_max,  
                    speed         = self._speed,          
                    reps_target   = self._reps_target     
                )
            except Exception as exc:
                import traceback
                traceback.print_exc()
                logger.error(f"start_session: create_session failed: {exc}")
                return {'ok': False, 'error': 'Database error creating session.'}

            # Tell serial_comm which session owns the port
            if self._serial is not None:
                self._serial.set_active_session_id(self._session_id)

            self._state = SessionState.STARTING

            log_system_event(
                'session', 'info',
                (
                    f"Session #{self._session_id} started. "
                    f"patient={patient_id}, mode={mode}, leg={leg}, "
                    f"hip_max={self._hip_flex_max}°, knee_max={self._knee_flex_max}°, "
                    f"speed={self._speed}°/s, reps={self._reps_target}"
                ),
                'session_manager',
            )

            self._launch_mode()

            return {
                'ok'         : True,
                'session_id' : self._session_id,
                'state'      : self._state.name,
            }

    def pause_session(self) -> dict:
        """
        Pause the running session. Motors hold current position.
        """
        with self._lock:
            if self._state != SessionState.RUNNING:
                return {'ok': False, 'error': 'No running session to pause.'}
            self._state = SessionState.PAUSED
            if self._mode_obj is not None:
                self._mode_obj.pause()
            logger.info(f"Session #{self._session_id} paused.")
            return {'ok': True, 'state': 'PAUSED'}

    def resume_session(self) -> dict:
        """
        Resume a paused session.
        """
        with self._lock:
            if self._state != SessionState.PAUSED:
                return {'ok': False, 'error': 'Session is not paused.'}
            self._state = SessionState.RUNNING
            if self._mode_obj is not None:
                self._mode_obj.resume()
            logger.info(f"Session #{self._session_id} resumed.")
            return {'ok': True, 'state': 'RUNNING'}

    def stop_session(self, status: str = 'completed') -> dict:
        """
        Stop the session cleanly. Flushes log buffer. Closes DB record.

        Args:
            status: 'completed' | 'interrupted' | 'estopped'
        """
        with self._lock:
            if self._session_id is None:
                return {'ok': False, 'error': 'No active session.'}
            if self._state in (SessionState.ENDED, SessionState.IDLE):
                return {'ok': False, 'error': 'Session already ended.'}

            sid           = self._session_id
            reps          = self._reps_done
            self._state   = SessionState.STOPPING
            mode_obj      = self._mode_obj
            self._mode_obj = None

        # Stop mode thread (outside lock to avoid deadlock)
        if mode_obj is not None:
            mode_obj.stop()

        # Flush remaining sensor data
        self._flush_buffer()

        # Close database record
        try:
            end_session(session_id=sid, status=status, reps_done=reps)
        except Exception as exc:
            logger.error(f"stop_session: end_session failed: {exc}")

        if self._serial is not None:
            self._serial.set_active_session_id(None)

        with self._lock:
            self._state      = SessionState.ENDED
            self._session_id = None

        log_system_event(
            'session', 'info',
            f"Session #{sid} ended. status={status}, reps={reps}",
            'session_manager',
        )

        return {'ok': True, 'status': status, 'reps_done': reps}

    def handle_estop(self) -> None:
        """
        Called immediately when E-stop fires (wired via main.py callback).
        Stops any running mode and marks session as estopped in DB.
        Does NOT re-arm the system — that is safety_monitor's responsibility.
        """
        with self._lock:
            if self._state in (
                SessionState.IDLE, SessionState.ENDED, SessionState.ESTOPPED
            ):
                return

            sid           = self._session_id
            reps          = self._reps_done
            self._state   = SessionState.ESTOPPED
            mode_obj      = self._mode_obj
            self._mode_obj = None

        logger.critical(
            f"SessionManager.handle_estop: stopping session #{sid}."
        )

        if mode_obj is not None:
            mode_obj.stop()

        self._flush_buffer()

        if sid is not None:
            try:
                end_session(session_id=sid, status='estopped', reps_done=reps)
                log_estop_event(
                    trigger_source = 'session_manager',
                    trigger_reason = 'E-stop received during active session',
                    session_id     = sid,
                )
            except Exception as exc:
                logger.error(f"handle_estop: DB write failed: {exc}")

        if self._serial is not None:
            self._serial.set_active_session_id(None)

    def update_rom(
        self,
        hip_flex_max  : float = None,
        knee_flex_max : float = None,
        ab_ad_max     : float = None,
        speed         : float = None,
        reps_target   : int   = None,
        pain_score    : int   = None,
    ) -> dict:
        """
        Update ROM parameters live during a running session.
        Called when therapist adjusts steppers on the therapy page.
        The active mode reads these via get_rom() on every cycle.

        All arguments are optional — only non-None values are updated.
        Returns the complete current ROM dict after update.
        """
        with self._lock:
            if hip_flex_max is not None:
                self._hip_flex_max  = float(hip_flex_max)
            if knee_flex_max is not None:
                self._knee_flex_max = float(knee_flex_max)
            if ab_ad_max is not None:
                self._ab_ad_max     = float(ab_ad_max)
            if speed is not None:
                self._speed = max(
                    CFG.passive_mode.min_speed_deg_per_sec,
                    min(CFG.passive_mode.max_speed_deg_per_sec, float(speed))
                )
            if reps_target is not None:
                self._reps_target   = int(reps_target)
            if pain_score is not None:
                self._pain_score    = int(pain_score)

            return self._rom_dict()

    # =========================================================================
    #  Public — called from therapy mode threads
    # =========================================================================

    def increment_reps(self) -> int:
        """
        Increment rep counter. Called by modes when one full cycle completes.
        Returns new reps_done count.
        """
        with self._lock:
            self._reps_done += 1
            logger.debug(
                f"Rep {self._reps_done}/{self._reps_target} completed."
            )
            return self._reps_done

    def is_reps_target_reached(self) -> bool:
        """Returns True if reps_done >= reps_target."""
        with self._lock:
            return self._reps_done >= self._reps_target

    def should_continue(self) -> bool:
        """
        Returns True if the mode loop should keep running.
        Modes call this at the top of every iteration.
        Returns False when state is STOPPING, ESTOPPED, or ENDED.
        """
        with self._lock:
            return self._state in (
                SessionState.RUNNING,
                SessionState.PAUSED,
                SessionState.STARTING,
            )

    def is_running(self) -> bool:
        """True if session is in RUNNING state (not paused)."""
        with self._lock:
            return self._state == SessionState.RUNNING

    def is_paused(self) -> bool:
        """True if session is PAUSED."""
        with self._lock:
            return self._state == SessionState.PAUSED

    def get_rom(self) -> dict:
        """
        Returns current ROM parameters dict.
        Called by mode threads on every cycle — must be fast.

        Keys:
            hip_flex_max  : float — degrees
            knee_flex_max : float — degrees
            ab_ad_max     : float — degrees
            speed         : float — degrees/sec
            reps_target   : int
        """
        with self._lock:
            return self._rom_dict()

    def log_sample(
        self,
        angles             : List[float],
        fsr_raw            : float,
        emg_raw            : float,
        emg_rms            : float,
        patient_effort_pct : float,
        robot_effort_pct   : float,
        intent_direction   : str,
    ) -> None:
        return
        """
        Buffer one sensor + state sample for async database write.

        Called by therapy modes at 20Hz (every waypoint send).
        The buffer is flushed to SQLite every flush_interval_sec seconds
        in the background — NOT on this call — to keep the therapy loop fast.

        Args:
            angles             : [ax1, ax2, ax3, ax4] in degrees
            fsr_raw            : Raw ADC from FSR (0–1023)
            emg_raw            : Raw ADC from EMG (0–1023)
            emg_rms            : Computed EMG RMS (0–1023 scale)
            patient_effort_pct : 0.0–100.0
            robot_effort_pct   : 0.0–100.0
            intent_direction   : 'PUSH' | 'LIFT' | 'NONE'
        """
        with self._lock:
            sid = self._session_id
        if sid is None:
            return

        sample = {
            'session_id'         : sid,
            'timestamp'          : time.time(),
            'ax1'                : float(angles[0]) if len(angles) > 0 else 0.0,
            'ax2'                : float(angles[1]) if len(angles) > 1 else 0.0,
            'ax3'                : float(angles[2]) if len(angles) > 2 else 0.0,
            'ax4'                : float(angles[3]) if len(angles) > 3 else 0.0,
            'fsr_raw'            : float(fsr_raw),
            'emg_raw'            : float(emg_raw),
            'emg_rms'            : float(emg_rms),
            'patient_effort_pct' : float(patient_effort_pct),
            'robot_effort_pct'   : float(robot_effort_pct),
            'intent_direction'   : str(intent_direction),
        }

        with self._log_lock:
            self._log_buffer.append(sample)
            if len(self._log_buffer) >= self._batch_size:
                self._flush_buffer_locked()

    # =========================================================================
    #  Status — read by ws.py and api.py
    # =========================================================================

    def get_status(self) -> dict:
        """
        Returns complete session status as JSON-serialisable dict.
        Called by ws.py at 20Hz and api.py for the status endpoint.
        """
        with self._lock:
            elapsed = (
                round(time.time() - self._start_time, 1)
                if self._start_time is not None else 0.0
            )
            return {
                'state'        : self._state.name,
                'session_id'   : self._session_id,
                'patient_id'   : self._patient_id,
                'mode'         : self._mode,
                'leg'          : self._leg,
                'reps_done'    : self._reps_done,
                'elapsed_sec'  : elapsed,
                'pain_score'   : self._pain_score,
                **self._rom_dict(),
            }

    # =========================================================================
    #  Internal
    # =========================================================================

    def _rom_dict(self) -> dict:
        """Returns ROM dict. Caller must hold self._lock."""
        return {
            'hip_flex_max'  : self._hip_flex_max,
            'knee_flex_max' : self._knee_flex_max,
            'ab_ad_max'     : self._ab_ad_max,
            'speed'         : self._speed,
            'reps_target'   : self._reps_target,
        }

    def _launch_mode(self) -> None:
        """
        Instantiate and start the correct therapy mode.
        Must be called while self._lock is held.
        Each mode runs on its own daemon thread.
        """
        if self._serial is None:
            # Dev mode — no hardware connected
            self._state = SessionState.RUNNING
            logger.warning(
                "SessionManager: dev mode — no serial port. "
                "Mode thread not launched."
            )
            return

        if self._mode == 'passive':
            from therapy.passive_mode import PassiveMode
            self._mode_obj = PassiveMode(
                serial_comm     = self._serial,
                sensor_hub      = self._sensor,
                session_manager = self,
            )
        elif self._mode == 'active':
            from therapy.active_assist import ActiveAssist
            self._mode_obj = ActiveAssist(
                serial_comm     = self._serial,
                sensor_hub      = self._sensor,
                session_manager = self,
            )
        elif self._mode == 'mimicry':
            from therapy.mimicry_mode import MimicryMode
            self._mode_obj = MimicryMode(
                serial_comm     = self._serial,
                sensor_hub      = self._sensor,
                session_manager = self,
            )

        if self._mode_obj is not None:
            self._mode_obj.start()
            self._state = SessionState.RUNNING

    # ── Log flush ──────────────────────────────────────────────────────────

    def _flush_loop(self) -> None:
        """
        Background thread — flushes log buffer to SQLite periodically.
        Runs until _flush_stop is set (by shutdown()).
        """
        while not self._flush_stop.wait(timeout=self._flush_interval):
            self._flush_buffer()

    def _flush_buffer(self) -> None:
        """Thread-safe flush."""
        with self._log_lock:
            self._flush_buffer_locked()

    def _flush_buffer_locked(self) -> None:
        """
        Write buffered samples to DB.
        Caller MUST hold self._log_lock.
        """
        if not self._log_buffer:
            return
        batch = list(self._log_buffer)
        self._log_buffer.clear()
        try:
            bulk_log_samples(batch)
        except Exception as exc:
            logger.error(f"_flush_buffer: bulk_log_samples failed: {exc}")
            # Return samples to buffer so they are not lost
            self._log_buffer = batch + self._log_buffer

    def shutdown(self) -> None:
        """
        Clean shutdown. Called by main.py on SIGINT / application exit.
        Stops any running session, flushes logs, stops flush thread.
        """
        logger.info("SessionManager: shutdown requested.")
        if self._session_id is not None:
            self.stop_session(status='interrupted')
        self._flush_stop.set()
        self._flush_thread.join(timeout=3.0)
        self._flush_buffer()
        logger.info("SessionManager: shutdown complete.")