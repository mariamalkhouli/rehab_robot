# =============================================================================
#  therapy/mimicry_mode.py
#  /home/rehabrobot/rehab_robot/therapy/mimicry_mode.py
#
#  Mimicry Mode — Teach & Replay
#
#  WHAT IT DOES:
#    Two sub-phases within one session:
#
#    TEACHING PHASE:
#      The therapist uses the jog buttons on the therapy page to position
#      the robot arm to a desired clinical pose. Each button press calls
#      serial_comm.send_jog(axis, steps) which moves the motor by exactly
#      jog_step_size_deg degrees. After positioning, the therapist presses
#      "Save Keyframe" — this reads the current motor angles from
#      serial_comm.get_current_angles() and stores them as a waypoint.
#      Building up >= 2 keyframes creates a "path".
#
#    REPLAY PHASE:
#      Once keyframes are saved, the therapist presses "Start Replay".
#      MimicryMode takes over from the SessionManager:
#        1. Validates all keyframes against joint limits
#        2. For each consecutive pair of keyframes, generates a smooth
#           interpolated trajectory (cosine profile, interpolation_steps points)
#        3. Sends each interpolated waypoint to the Arduino at 20Hz
#        4. Holds at each keyframe for keyframe.hold_time_ms milliseconds
#        5. Loops the entire path replay_loops times (0 = infinite)
#        6. Calls session_manager.increment_reps() after each full path loop
#
#  KEYFRAME STORAGE:
#    Keyframes are stored in the SQLite database via the therapy_paths and
#    keyframes tables. The path_id is created at session start. After the
#    session ends, the path is permanently saved and can be replayed in
#    future sessions.
#
#    DB functions used:
#      create_therapy_path(patient_id, created_by, name, target_joints,
#                          speed, loop_count)  → path_id
#      save_keyframe(path_id, order_index, ax1, ax2, ax3, ax4,
#                    hold_time_ms, interp_speed)  → keyframe_id
#      get_keyframes_for_path(path_id)  → List[sqlite3.Row]
#
#  SAFETY VALIDATION BEFORE REPLAY:
#    validate_keyframes() runs kinematics.validate_angles() on every saved
#    keyframe. If ANY keyframe has an angle outside safe limits, replay is
#    refused and a clear error is returned. This is belt-and-suspenders on
#    top of serial_comm's own clamping.
#
#  THREADING:
#    MimicryMode has TWO operational states (not a separate teaching thread):
#      - TEACHING: No motor loop thread runs. Motors are commanded directly
#        by jog calls from api.py (from the Flask thread). MimicryMode just
#        manages keyframe storage.
#      - REPLAYING: start_replay() launches the "MimicryReplay" daemon thread
#        which executes the full path loop.
#
#    This is different from PassiveMode and ActiveAssist which start their
#    loop immediately when start() is called. MimicryMode.start() puts the
#    mode into TEACHING state. start_replay() transitions to REPLAYING state.
#
#  CONFIG KEYS USED (exact names from config.yaml):
#    CFG.mimicry_mode.jog_step_size_deg       = 1.0
#    CFG.mimicry_mode.jog_speed_deg_per_sec   = 10.0
#    CFG.mimicry_mode.max_keyframes_per_path  = 100
#    CFG.mimicry_mode.interpolation_steps     = 50
#    CFG.mimicry_mode.replay_speed_multiplier = 1.0
#    CFG.mimicry_mode.min_replay_speed        = 0.25
#    CFG.mimicry_mode.max_replay_speed        = 2.0
#    CFG.serial.send_rate_hz                  = 20
#    CFG.passive_mode.trajectory_points       = 200  (for return-to-home)
# =============================================================================

import time
import threading
import logging
from enum import Enum, auto
from typing import List, Optional, Tuple

from core.config import CFG
from core.database import (
    create_therapy_path,
    save_keyframe,
    get_keyframes_for_path,
)
from therapy.kinematics import get_engine, JointAngles

logger = logging.getLogger(__name__)

# Send period for replay loop
_SEND_PERIOD_SEC = 1.0 / CFG.serial.send_rate_hz   # 0.05s at 20Hz


# =============================================================================
#  MimicryState — internal state of this mode
# =============================================================================

class MimicryState(Enum):
    IDLE      = auto()   # Mode object created, not yet started
    TEACHING  = auto()   # Therapist is jogging and saving keyframes
    REPLAYING = auto()   # Replay thread is running
    STOPPING  = auto()   # Stop requested, winding down
    STOPPED   = auto()   # Fully stopped


# =============================================================================
#  Keyframe data class
# =============================================================================

class Keyframe:
    """
    One recorded motor position in a teaching path.

    Attributes:
        idx         : Order index (0-based)
        angles      : JointAngles at this keyframe
        hold_time_ms: How long to hold at this position before moving on
        interp_speed: Override replay speed for segment after this keyframe.
                      None = use path-level speed.
    """
    def __init__(self, idx: int, angles: JointAngles,
                 hold_time_ms: float = 0.0,
                 interp_speed: Optional[float] = None):
        self.idx          = idx
        self.angles       = angles
        self.hold_time_ms = hold_time_ms
        self.interp_speed = interp_speed

    def __repr__(self):
        return (f"Keyframe({self.idx}: "
                f"M1={self.angles.ax1:.1f}°, M2={self.angles.ax2:.1f}°, "
                f"M3={self.angles.ax3:.1f}°)")


# =============================================================================
#  MimicryMode
# =============================================================================

class MimicryMode:
    """
    Mimicry (Teach & Replay) therapy mode.

    Instantiated by SessionManager._launch_mode() when mode == 'mimicry'.
    Unlike passive and active modes, this mode has two phases:
      1. Teaching phase  — therapist jogs motors via the UI
      2. Replay phase    — robot replays the taught path

    The SessionManager calls:
      start()        → enters TEACHING state, no thread yet
      stop()         → cleans up everything
      pause()        → pauses replay if running
      resume()       → resumes replay

    The API endpoint (api.py) calls:
      add_keyframe() → saves current motor position as a keyframe
      start_replay() → validates keyframes and launches replay thread
      clear_keyframes() → deletes all keyframes for this session path

    Args:
        serial_comm     : core.serial_comm.SerialComm instance
        sensor_hub      : core.sensor_hub.SensorHub instance (may be None)
        session_manager : therapy.session_manager.SessionManager instance
    """

    def __init__(self, serial_comm, sensor_hub, session_manager):
        self._serial  = serial_comm
        self._sensor  = sensor_hub
        self._mgr     = session_manager
        self._engine  = get_engine()

        # Internal state
        self._state        : MimicryState  = MimicryState.IDLE
        self._state_lock   = threading.Lock()

        # Keyframe storage — in memory during the session
        self._keyframes    : List[Keyframe] = []
        self._kf_lock      = threading.Lock()

        # Database path record — created when first keyframe is saved
        self._path_id      : Optional[int]  = None

        # Replay thread controls
        self._stop_event   = threading.Event()
        self._pause_event  = threading.Event()
        self._replay_thread: Optional[threading.Thread] = None

        # Replay speed multiplier — can be changed live by therapist
        self._speed_mult   : float = CFG.mimicry_mode.replay_speed_multiplier

        logger.info("MimicryMode: initialised.")

    # =========================================================================
    #  Public interface — called by SessionManager (Flask thread)
    # =========================================================================

    def start(self) -> None:
        """
        Enter TEACHING state. No replay thread is started yet.
        The therapist uses jog buttons on the UI to position the arm.
        """
        with self._state_lock:
            self._state = MimicryState.TEACHING

        # Set kinematics engine side for M4 computation
        self._engine.set_affected_side(self._mgr._leg)

        logger.info(
            "MimicryMode: TEACHING state. "
            "Therapist may now jog axes and save keyframes."
        )

    def pause(self) -> None:
        """Pause replay. Motors hold current position."""
        logger.info("MimicryMode: replay paused.")
        self._pause_event.set()

    def resume(self) -> None:
        """Resume replay after pause."""
        logger.info("MimicryMode: replay resumed.")
        self._pause_event.clear()

    def stop(self) -> None:
        """
        Stop everything — teaching or replay.
        Called by session_manager.stop_session() or handle_estop().
        """
        logger.info("MimicryMode: stop requested.")
        with self._state_lock:
            self._state = MimicryState.STOPPING

        self._stop_event.set()
        self._pause_event.clear()

        if self._replay_thread is not None and self._replay_thread.is_alive():
            self._replay_thread.join(timeout=5.0)
            if self._replay_thread.is_alive():
                logger.warning("MimicryMode: replay thread did not exit in 5s.")

        with self._state_lock:
            self._state = MimicryState.STOPPED

        logger.info("MimicryMode: stopped.")

    # =========================================================================
    #  Keyframe management — called from api.py (Flask thread)
    # =========================================================================

    def add_keyframe(
        self,
        hold_time_ms : float = 0.0,
        interp_speed : Optional[float] = None,
    ) -> dict:
        """
        Record the current motor position as the next keyframe.

        Reads current angles from serial_comm.get_current_angles(),
        validates them, and appends to the in-memory keyframes list.
        Also writes to the database for permanent storage.

        Args:
            hold_time_ms  : How long to hold at this position during replay (ms)
            interp_speed  : Override speed for the segment AFTER this keyframe.
                            None = use path-level speed from session ROM.

        Returns:
            dict with keyframe index, angles, and db keyframe_id
            or error key if failed.
        """
        with self._state_lock:
            if self._state != MimicryState.TEACHING:
                return {
                    'ok': False,
                    'error': 'Cannot add keyframe: not in teaching state.'
                }

        # Check limit
        with self._kf_lock:
            if len(self._keyframes) >= CFG.mimicry_mode.max_keyframes_per_path:
                return {
                    'ok': False,
                    'error': f"Maximum keyframes ({CFG.mimicry_mode.max_keyframes_per_path}) reached."
                }

        # Read current motor angles
        if self._serial is None:
            # Dev mode — use zeros
            current_list = [0.0, 0.0, 0.0, 0.0]
        else:
            current_list = self._serial.get_current_angles()

        angles = JointAngles.from_list(current_list)

        # Validate against safe limits before saving
        valid, reason = self._engine.validate_angles(angles)
        if not valid:
            logger.warning(f"MimicryMode.add_keyframe: invalid angles: {reason}")
            return {'ok': False, 'error': f"Unsafe position: {reason}"}

        # Create the DB path record on first keyframe
        if self._path_id is None:
            self._path_id = self._create_db_path()

        # Save to database
        with self._kf_lock:
            idx = len(self._keyframes)

        kf_db_id = None
        if self._path_id is not None:
            try:
                kf_db_id = save_keyframe(
                    path_id      = self._path_id,
                    order_index  = idx,
                    ax1          = angles.ax1,
                    ax2          = angles.ax2,
                    ax3          = angles.ax3,
                    ax4          = angles.ax4,
                    hold_time_ms = hold_time_ms,
                    interp_speed = interp_speed,
                )
            except Exception as exc:
                logger.error(f"MimicryMode.add_keyframe: DB write failed: {exc}")

        # Store in memory
        kf = Keyframe(
            idx          = idx,
            angles       = angles,
            hold_time_ms = hold_time_ms,
            interp_speed = interp_speed,
        )
        with self._kf_lock:
            self._keyframes.append(kf)

        logger.info(
            f"MimicryMode: keyframe {idx} saved — {angles}"
        )

        return {
            'ok'          : True,
            'keyframe_idx': idx,
            'keyframe_id' : kf_db_id,
            'angles'      : {
                'ax1': round(angles.ax1, 2),
                'ax2': round(angles.ax2, 2),
                'ax3': round(angles.ax3, 2),
                'ax4': round(angles.ax4, 2),
            },
        }

    def clear_keyframes(self) -> dict:
        """
        Delete all in-memory keyframes for this session.
        The DB records are kept (soft-delete is on the path, not keyframes).

        Returns:
            dict with count of keyframes cleared.
        """
        with self._kf_lock:
            count = len(self._keyframes)
            self._keyframes.clear()

        logger.info(f"MimicryMode: {count} keyframes cleared.")
        return {'ok': True, 'cleared': count}

    def get_keyframes(self) -> List[dict]:
        """
        Returns all saved keyframes as a JSON-serializable list.
        Called by api.py for the therapy page keyframe display.
        """
        with self._kf_lock:
            return [
                {
                    'idx'          : kf.idx,
                    'ax1'          : round(kf.angles.ax1, 2),
                    'ax2'          : round(kf.angles.ax2, 2),
                    'ax3'          : round(kf.angles.ax3, 2),
                    'ax4'          : round(kf.angles.ax4, 2),
                    'hold_time_ms' : kf.hold_time_ms,
                }
                for kf in self._keyframes
            ]

    # =========================================================================
    #  Replay
    # =========================================================================

    def start_replay(
        self,
        replay_loops      : int   = 0,
        speed_multiplier  : float = None,
    ) -> dict:
        """
        Validate keyframes and start the replay thread.

        Args:
            replay_loops     : How many times to loop the path.
                               0 = loop until reps_target is reached.
                               session_manager.is_reps_target_reached() stops it.
            speed_multiplier : Scale replay speed. 1.0 = normal.
                               Clamped to [min_replay_speed, max_replay_speed].

        Returns:
            {'ok': True, 'keyframe_count': N} if started
            {'ok': False, 'error': reason} if validation fails
        """
        with self._state_lock:
            if self._state != MimicryState.TEACHING:
                return {
                    'ok': False,
                    'error': f"Cannot start replay: current state is {self._state.name}"
                }

        # Need at least 2 keyframes to define a path
        with self._kf_lock:
            kf_count = len(self._keyframes)
            kf_copy  = list(self._keyframes)   # snapshot for thread safety

        if kf_count < 2:
            return {
                'ok': False,
                'error': f"Need at least 2 keyframes. Only {kf_count} saved."
            }

        # Validate ALL keyframes before starting
        valid, reason = self._validate_all_keyframes(kf_copy)
        if not valid:
            return {'ok': False, 'error': f"Keyframe validation failed: {reason}"}

        # Set speed multiplier
        if speed_multiplier is not None:
            self._speed_mult = max(
                CFG.mimicry_mode.min_replay_speed,
                min(CFG.mimicry_mode.max_replay_speed, float(speed_multiplier))
            )

        # Transition to replay state
        with self._state_lock:
            self._state = MimicryState.REPLAYING

        self._stop_event.clear()
        self._pause_event.clear()

        # Launch replay thread
        self._replay_thread = threading.Thread(
            target = self._replay_loop,
            args   = (kf_copy, replay_loops),
            name   = "MimicryReplay",
            daemon = True,
        )
        self._replay_thread.start()

        logger.info(
            f"MimicryMode: replay started. "
            f"{kf_count} keyframes, "
            f"loops={replay_loops if replay_loops > 0 else 'until reps target'}, "
            f"speed_mult={self._speed_mult}"
        )

        return {'ok': True, 'keyframe_count': kf_count}

    def set_speed_multiplier(self, multiplier: float) -> None:
        """
        Adjust replay speed live during replay.
        Therapist can call this from the UI to go faster or slower.
        """
        self._speed_mult = max(
            CFG.mimicry_mode.min_replay_speed,
            min(CFG.mimicry_mode.max_replay_speed, float(multiplier))
        )
        logger.info(f"MimicryMode: speed multiplier set to {self._speed_mult}")

    # =========================================================================
    #  Replay loop — runs on MimicryReplay thread
    # =========================================================================

    def _replay_loop(
        self,
        keyframes    : List[Keyframe],
        replay_loops : int,
    ) -> None:
        """
        Execute the recorded path by interpolating between keyframes.

        Each "loop" = playing the path from keyframe[0] to keyframe[N-1].
        After each loop, session_manager.increment_reps() is called.
        If reps_target is reached or stop_event is set, the loop exits.

        Args:
            keyframes    : Snapshot of keyframes taken at start_replay()
            replay_loops : Max loops. 0 = unlimited (stops on reps_target).
        """
        logger.info(f"MimicryMode: replay loop started ({len(keyframes)} keyframes).")
        loop_count = 0

        while True:

            # ── Stop guard ────────────────────────────────────────────────
            if self._stop_event.is_set():
                logger.info("MimicryMode: stop event — exiting replay.")
                break

            if not self._mgr.should_continue():
                logger.info("MimicryMode: session_manager says stop.")
                break

            if self._mgr.is_reps_target_reached():
                logger.info("MimicryMode: reps target reached — ending session.")
                self._mgr.stop_session(status='completed')
                return

            if replay_loops > 0 and loop_count >= replay_loops:
                logger.info(f"MimicryMode: {replay_loops} replay loops completed.")
                break

            # ── Execute one full path traversal ───────────────────────────
            ok = self._execute_path(keyframes)
            if not ok:
                break

            # ── Count this loop as one rep ────────────────────────────────
            loop_count += 1
            reps = self._mgr.increment_reps()
            logger.info(
                f"MimicryMode: loop {loop_count} complete "
                f"(rep {reps}/{self._mgr.get_rom()['reps_target']})."
            )

        # ── Return to home on normal exit ──────────────────────────────────
        if not self._serial.is_halted():
            self._return_to_home()

        with self._state_lock:
            if self._state == MimicryState.REPLAYING:
                self._state = MimicryState.TEACHING   # Back to teaching state

        logger.info("MimicryMode: replay loop exited.")

    def _execute_path(self, keyframes: List[Keyframe]) -> bool:
        """
        Execute one traversal of the full keyframe path.

        Iterates through consecutive keyframe pairs and for each segment:
          1. Generates an interpolated trajectory (cosine profile)
          2. Sends each waypoint at send_rate_hz
          3. Holds at the destination keyframe for hold_time_ms

        Args:
            keyframes : The keyframe list (snapshot, thread-safe)

        Returns:
            True  — path completed normally
            False — interrupted by stop or halt
        """
        for i in range(len(keyframes) - 1):
            if self._stop_event.is_set():
                return False
            if not self._mgr.should_continue():
                return False

            kf_start = keyframes[i]
            kf_end   = keyframes[i + 1]

            # Determine speed for this segment
            # Priority: keyframe-specific override → path-level speed → default
            rom   = self._mgr.get_rom()
            speed = kf_start.interp_speed or rom['speed']
            speed = speed * self._speed_mult
            speed = max(0.5, speed)   # floor at 0.5°/s to prevent stall

            # Build interpolated trajectory between the two keyframes
            trajectory = self._engine.generate_trajectory(
                start    = kf_start.angles,
                end      = kf_end.angles,
                n_points = CFG.mimicry_mode.interpolation_steps,
                profile  = 'cosine',
            )

            # Compute timing
            # Use the larger of hip or knee range to determine duration
            ax2_range = abs(kf_end.angles.ax2 - kf_start.angles.ax2)
            ax3_range = abs(kf_end.angles.ax3 - kf_start.angles.ax3)
            ax1_range = abs(kf_end.angles.ax1 - kf_start.angles.ax1)
            max_range = max(ax2_range, ax3_range, ax1_range, 1.0)

            total_time_sec = max_range / speed
            n_points       = len(trajectory)
            dt             = max(total_time_sec / n_points, _SEND_PERIOD_SEC)

            # Send each waypoint
            for wp in trajectory:
                loop_start = time.monotonic()

                if self._stop_event.is_set():
                    return False

                if not self._mgr.should_continue():
                    return False

                # Pause handling
                if self._pause_event.is_set():
                    while self._pause_event.is_set():
                        if self._stop_event.is_set():
                            return False
                        self._hold_position()
                        time.sleep(0.05)

                # Hardware halt check
                if self._serial.is_halted():
                    logger.warning("MimicryMode: serial halted — waiting.")
                    while self._serial.is_halted():
                        if self._stop_event.is_set():
                            return False
                        time.sleep(0.1)
                    continue

                # Send waypoint
                angles_list = wp.to_list()
                self._serial.send_angles(angles_list)

                # Log sample
                self._log_sample(angles_list)

                # Sleep remainder
                elapsed    = time.monotonic() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

            # Hold at end of this segment (at kf_end position)
            if kf_end.hold_time_ms > 0:
                ok = self._interruptible_hold(kf_end.hold_time_ms)
                if not ok:
                    return False

        return True

    # =========================================================================
    #  Validation
    # =========================================================================

    def _validate_all_keyframes(
        self, keyframes: List[Keyframe]
    ) -> Tuple[bool, str]:
        """
        Validate every keyframe against joint safe limits.
        Called before starting replay to prevent unsafe path execution.

        Returns:
            (True, "")           if all keyframes are safe
            (False, reason_str)  if any keyframe is out of range
        """
        for kf in keyframes:
            valid, reason = self._engine.validate_angles(kf.angles)
            if not valid:
                msg = f"Keyframe {kf.idx} unsafe: {reason}"
                logger.error(f"MimicryMode: {msg}")
                return False, msg

        # Also check that no two consecutive keyframes require a step larger
        # than what is physically reasonable (e.g. 90° instant jump would be
        # a sign of a data error, not a valid clinical path)
        max_step_deg = 60.0   # degrees — large enough for real paths
        for i in range(len(keyframes) - 1):
            a = keyframes[i].angles
            b = keyframes[i+1].angles
            for attr in ['ax1', 'ax2', 'ax3']:
                delta = abs(getattr(b, attr) - getattr(a, attr))
                if delta > max_step_deg:
                    msg = (
                        f"Keyframe {i}→{i+1}: {attr} jump of {delta:.1f}° "
                        f"exceeds max_step_deg ({max_step_deg}°). "
                        f"Possible data error — check keyframes."
                    )
                    logger.warning(f"MimicryMode: {msg}")
                    # This is a warning, not a hard block — therapist may
                    # intentionally have large jumps. Log but allow.

        return True, ""

    # =========================================================================
    #  Database helpers
    # =========================================================================

    def _create_db_path(self) -> Optional[int]:
        """
        Create a therapy_paths record for this session's taught path.
        Called once when the first keyframe is saved.

        Returns the path_id, or None if DB write fails.
        """
        try:
            rom = self._mgr.get_rom()
            path_id = create_therapy_path(
                patient_id    = self._mgr._patient_id,
                created_by    = None,    # TODO: wire therapist_id when auth is added
                name          = f"Session #{self._mgr._session_id} Path",
                target_joints = 'hip_flex,knee_flex',
                speed         = rom['speed'],
                loop_count    = rom['reps_target'],
            )
            logger.info(f"MimicryMode: therapy path #{path_id} created in DB.")
            return path_id
        except Exception as exc:
            logger.error(f"MimicryMode._create_db_path: DB write failed: {exc}")
            return None

    # =========================================================================
    #  Position helpers
    # =========================================================================

    def _hold_position(self) -> None:
        """
        Re-send current motor angles to prevent gravity-induced drift.
        Called during pause and at keyframe hold positions.
        """
        if self._serial is None or self._serial.is_halted():
            return
        current = self._serial.get_current_angles()
        self._serial.send_angles(current)

    def _interruptible_hold(self, duration_ms: float) -> bool:
        """
        Hold current position for duration_ms milliseconds.
        Sliced into 50ms intervals — interruptible by stop_event.

        Args:
            duration_ms: Hold duration in milliseconds

        Returns:
            True  — hold completed normally
            False — interrupted by stop_event
        """
        if duration_ms <= 0:
            return True

        deadline = time.monotonic() + duration_ms / 1000.0

        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            if not self._mgr.should_continue():
                return False
            self._hold_position()
            time.sleep(0.05)

        return True

    def _return_to_home(self) -> None:
        """
        Smoothly move the arm to home position (all axes at 0°).
        Called at the end of completed replay or after stop.
        """
        logger.info("MimicryMode: returning to home.")

        if self._serial is None:
            return

        current_list = self._serial.get_current_angles()
        current      = JointAngles.from_list(current_list)
        home         = JointAngles(ax1=0.0, ax2=0.0, ax3=0.0, ax4=0.0)

        if abs(current.ax2) < 0.5 and abs(current.ax3) < 0.5:
            logger.info("MimicryMode: already at home — skip.")
            return

        return_traj = self._engine.generate_trajectory(
            start    = current,
            end      = home,
            n_points = CFG.passive_mode.trajectory_points // 2,
            profile  = 'cosine',
        )

        for wp in return_traj:
            if self._stop_event.is_set():
                break
            if self._serial.is_halted():
                break
            self._serial.send_angles(wp.to_list())
            time.sleep(_SEND_PERIOD_SEC)

        logger.info("MimicryMode: home reached.")

    # =========================================================================
    #  Sensor logging
    # =========================================================================

    def _log_sample(self, angles_list: List[float]) -> None:
        """
        Buffer one sensor sample during replay.
        In mimicry mode, patient effort is not the primary metric —
        the robot is executing a pre-recorded path. We still log sensors
        for clinical records (patient co-contraction, comfort level).
        """
        if self._sensor is None:
            self._mgr.log_sample(
                angles             = angles_list,
                fsr_raw            = 0.0,
                emg_raw            = 0.0,
                emg_rms            = 0.0,
                patient_effort_pct = 0.0,
                robot_effort_pct   = 100.0,
                intent_direction   = 'NONE',
            )
            return

        raw                = self._sensor.get_raw()
        emg_rms            = self._sensor.get_emg_rms()
        patient_effort_pct = self._sensor.get_patient_effort_pct()
        direction, _mag    = self._sensor.get_intent()

        self._mgr.log_sample(
            angles             = angles_list,
            fsr_raw            = raw['fsr_raw'],
            emg_raw            = raw['emg_raw'],
            emg_rms            = emg_rms,
            patient_effort_pct = patient_effort_pct,
            robot_effort_pct   = 100.0,
            intent_direction   = direction,
        )

    # =========================================================================
    #  Status — read by api.py and ws.py
    # =========================================================================

    def get_status(self) -> dict:
        """
        Returns current mimicry state as JSON-serializable dict.
        Called by api.py when the therapy page requests mimicry status.
        """
        with self._state_lock:
            state_name = self._state.name

        with self._kf_lock:
            kf_count = len(self._keyframes)

        return {
            'mimicry_state'   : state_name,
            'keyframe_count'  : kf_count,
            'max_keyframes'   : CFG.mimicry_mode.max_keyframes_per_path,
            'path_id'         : self._path_id,
            'speed_multiplier': self._speed_mult,
            'can_replay'      : kf_count >= 2 and state_name == 'TEACHING',
        }