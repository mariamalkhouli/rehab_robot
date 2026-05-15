# =============================================================================
#  therapy/passive_mode.py
#  /home/rehabrobot/rehab_robot/therapy/passive_mode.py
#
#  Passive CPM Mode — Continuous Passive Motion
#
#  WHAT IT DOES:
#    Drives the robot arm through a smooth pre-computed trajectory repeatedly.
#    The patient does nothing — the robot does all the work.
#    This is the baseline exercise at the start of rehabilitation.
#
#  ALGORITHM (one repetition):
#    1. Read ROM targets from session_manager.get_rom()
#    2. Build outward trajectory: home → max flex (cosine S-curve)
#    3. Send each waypoint to Arduino via serial_comm.send_angles() at 20Hz
#    4. Hold at max flexion for hold_time_at_limits_ms ms
#    5. Build return trajectory: max flex → home
#    6. Send return sweep at same rate
#    7. Hold at home for hold_time_at_limits_ms ms
#    8. Call session_manager.increment_reps()
#    9. Repeat from step 1 — ROM targets re-read each rep (live adjustment)
#
#  SEND RATE:
#    Waypoints are sent at CFG.serial.send_rate_hz = 20Hz (50ms interval).
#    serial_comm.send_angles() is rate-limited internally — safe to call faster.
#
#  MOTION PROFILE:
#    Cosine S-curve (profile='cosine') — zero velocity at start and end
#    of each sweep. This eliminates jerk that triggers spastic catch in
#    stroke patients. Confirmed as clinical default in project design.
#
#  EXERCISE:
#    Combined hip + knee flexion (knee toward chest). Both axes move
#    simultaneously and proportionally. M4 (cuff alignment) is computed
#    automatically by kinematics.fill_m4() on every waypoint.
#
#  SAFETY:
#    - All angles clamped inside serial_comm.send_angles() (joint limits)
#    - kinematics.clamp_to_safe_limits() applied by generate_trajectory()
#    - Checks serial_comm.is_halted() every waypoint
#    - Checks session_manager.should_continue() every waypoint
#    - Pause is interruptible — checks every 50ms during hold
#    - Stale sensor data is tolerated in passive mode (no sensor control)
#
#  THREADING:
#    Runs on daemon thread "PassiveCPM".
#    pause() / resume() / stop() called from Flask HTTP thread — safe.
#
#  CONFIG KEYS USED:
#    CFG.serial.send_rate_hz                    = 20
#    CFG.passive_mode.hold_time_at_limits_ms    = 500
#    CFG.passive_mode.trajectory_points         = 200
#    CFG.passive_mode.min_speed_deg_per_sec     = 1.0
#    CFG.passive_mode.max_speed_deg_per_sec     = 15.0
#    CFG.passive_mode.default_speed_deg_per_sec = 5.0
# =============================================================================

import time
import threading
import logging
from typing import List, Tuple

from core.config import CFG
from therapy.kinematics import get_engine, JointAngles

logger = logging.getLogger(__name__)

# Send period derived from config — used for timing calculations
_SEND_PERIOD_SEC = 1.0 / CFG.serial.send_rate_hz  # 0.05s at 20Hz


class PassiveMode:
    """
    Continuous Passive Motion therapy mode.

    Instantiated by SessionManager._launch_mode() when mode == 'passive'.
    Runs on its own daemon thread. Session manager holds the reference
    and calls stop() when the session ends or E-stop fires.

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

        # Control events
        self._stop_event  = threading.Event()  # set → exit immediately
        self._pause_event = threading.Event()  # set → hold position

        # Worker thread
        self._thread = threading.Thread(
            target = self._run,
            name   = "PassiveCPM",
            daemon = True,
        )

        logger.info("PassiveMode: initialised.")

    # =========================================================================
    #  Public interface — called by SessionManager (Flask thread)
    # =========================================================================

    def start(self) -> None:
        """Launch the CPM thread. Called once by SessionManager."""
        logger.info("PassiveMode: starting.")
        self._thread.start()

    def pause(self) -> None:
        """Pause motion. Motors hold current position via position re-sends."""
        logger.info("PassiveMode: paused.")
        self._pause_event.set()

    def resume(self) -> None:
        """Resume motion after pause."""
        logger.info("PassiveMode: resumed.")
        self._pause_event.clear()

    def stop(self) -> None:
        """
        Signal thread to stop and block until it exits.
        Called by session_manager.stop_session() or handle_estop().
        Safe to call from any thread.
        """
        logger.info("PassiveMode: stop requested.")
        self._stop_event.set()
        self._pause_event.clear()        # Unblock if currently paused
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            logger.warning(
                "PassiveMode: thread did not exit within 5s — "
                "may have been blocked on serial write."
            )
        else:
            logger.info("PassiveMode: thread stopped cleanly.")

    # =========================================================================
    #  Main loop — runs on PassiveCPM thread
    # =========================================================================

    def _run(self) -> None:
        """
        CPM execution loop.

        Outer loop  = one repetition per iteration.
        Inner loops = one waypoint per iteration (inside _execute_sweep).
        """
        logger.info("PassiveMode: loop started.")

        # Set the kinematics engine's side sign for left/right leg
        self._engine.set_affected_side(self._mgr._leg)

        while True:

            # ── Guard: stop requested ──────────────────────────────────────
            if self._stop_event.is_set():
                logger.info("PassiveMode: stop event set — exiting loop.")
                break

            # ── Guard: session should not continue ─────────────────────────
            if not self._mgr.should_continue():
                logger.info("PassiveMode: session_manager says stop.")
                break

            # ── Guard: reps target reached ─────────────────────────────────
            if self._mgr.is_reps_target_reached():
                logger.info(
                    "PassiveMode: reps target reached — ending session."
                )
                self._mgr.stop_session(status='completed')
                return   # session_manager handles cleanup — just exit

            # ── Read live ROM targets ──────────────────────────────────────
            # Re-read at every rep start so therapist live adjustments
            # take effect at the beginning of the next cycle.
            rom      = self._mgr.get_rom()
            hip_max  = rom['hip_flex_max']
            knee_max = rom['knee_flex_max']
            speed    = max(
                CFG.passive_mode.min_speed_deg_per_sec,
                min(CFG.passive_mode.max_speed_deg_per_sec, rom['speed'])
            )

            logger.debug(
                f"PassiveMode: rep start — "
                f"hip_max={hip_max:.1f}°, knee_max={knee_max:.1f}°, "
                f"speed={speed:.1f}°/s"
            )

            # ── Build outward and return trajectories ──────────────────────
            outward, inward = self._build_cycle_trajectories(
                hip_max  = hip_max,
                knee_max = knee_max,
                speed    = speed,
            )

            # ── Execute outward sweep: home → max flex ─────────────────────
            ok = self._execute_sweep(outward, speed, hip_max, knee_max)
            if not ok:
                break

            # ── Hold at maximum flexion ────────────────────────────────────
            ok = self._interruptible_hold(CFG.passive_mode.hold_time_at_limits_ms)
            if not ok:
                break

            # ── Execute return sweep: max flex → home ──────────────────────
            ok = self._execute_sweep(inward, speed, hip_max, knee_max)
            if not ok:
                break

            # ── Hold at home position ──────────────────────────────────────
            ok = self._interruptible_hold(CFG.passive_mode.hold_time_at_limits_ms)
            if not ok:
                break

            # ── Count this completed repetition ───────────────────────────
            reps_done = self._mgr.increment_reps()
            logger.info(
                f"PassiveMode: rep {reps_done}/{rom['reps_target']} complete."
            )

        # ── Smooth return to home on exit ──────────────────────────────────
        # Only if session ended normally (not E-stop / serial halt)
        if not self._serial.is_halted():
            self._return_to_home()

        logger.info("PassiveMode: loop exited.")

    # =========================================================================
    #  Trajectory building
    # =========================================================================

    def _build_cycle_trajectories(
            self,
            hip_max  : float,
            knee_max : float,
            speed    : float,
        ) -> Tuple[List[JointAngles], List[JointAngles]]:
            """
            Build trajectories based on the specific clinical exercise chosen.
            """
            n_points = CFG.passive_mode.trajectory_points
            home = JointAngles(ax1=0.0, ax2=0.0, ax3=0.0, ax4=0.0)

            # Get the exercise ID from the manager (set by the web dropdown)
            # IDs: 'hip_ab_ad', 'hip_flex_ext', 'knee_flex_ext'
            exercise_type = getattr(self._mgr, '_exercise', 'hip_flex_ext')

            if exercise_type == "hip_ab_ad":
                # --- EXERCISE 1: SIDEYWAYS (Pivot) ---
                # Moves ONLY Axis 1. Hip/Knee/Cuff stay at 0.
                peak = JointAngles(ax1=self._mgr._ab_ad_max, ax2=0.0, ax3=0.0)
                logger.info(f"Exercise: Hip Abduction to {self._mgr._ab_ad_max}°")
            
            elif exercise_type == "hip_flex_ext":
                # --- EXERCISE 2: UP/DOWN (Straight Leg Raise) ---
                # Moves ONLY Axis 2. Knee (ax3) is locked at 0.
                # M4 (ax4) will auto-calculate in the kinematics engine.
                peak = JointAngles(ax1=0.0, ax2=hip_max, ax3=0.0)
                logger.info(f"Exercise: Hip Flexion (SLR) to {hip_max}°")

            elif exercise_type == "knee_flex_ext":
                # --- EXERCISE 3: KNEE BEND (Coupled) ---
                # Moves Axis 3. Axis 2 follows at 20% to lift the thigh
                # so the patient's heel doesn't hit the bed.
                peak = JointAngles(ax1=0.0, ax2=knee_max * 0.2, ax3=knee_max)
                logger.info(f"Exercise: Knee Flexion to {knee_max}° (Hip follower active)")

            else:
                # Fallback
                peak = JointAngles(ax1=0.0, ax2=0.0, ax3=0.0)

            # Generate trajectories. 
            # Note: generate_trajectory calls clamp_to_safe_limits internally, 
            # which now handles the Motor 4 orientation automatically!
            outward = self._engine.generate_trajectory(home, peak, n_points, 'cosine')
            inward  = self._engine.generate_trajectory(peak, home, n_points, 'cosine')

            return outward, inward

    # =========================================================================
    #  Sweep execution
    # =========================================================================

    def _execute_sweep(
        self,
        waypoints : List[JointAngles],
        speed     : float,
        hip_max   : float,
        knee_max  : float,
    ) -> bool:
        """
        Send a list of waypoints to the Arduino one by one.

        Timing:
            The inter-waypoint delay (dt) is calculated from speed so that
            the joint travels at approximately speed degrees/second.
            Reference axis: whichever of hip_max or knee_max is larger.

            dt = (range_degrees / speed_deg_per_sec) / n_points
            dt = max(dt, _SEND_PERIOD_SEC) — never faster than serial rate

        Pause:
            If pause_event is set, holds position by re-sending current
            angles every 50ms until resumed or stopped.

        Halt:
            If serial is halted (E-stop), waits in a tight loop every 100ms.
            Does not force-exit — session_manager.handle_estop() will
            set the stop_event and call our stop() shortly after.

        Args:
            waypoints : Pre-computed from generate_trajectory()
            speed     : degrees/sec — used to compute dt
            hip_max   : Used to compute angle range for timing
            knee_max  : Used to compute angle range for timing

        Returns:
            True  — sweep completed fully
            False — interrupted by stop or unrecoverable condition
        """
        if not waypoints:
            return True

        # Compute time per waypoint based on the larger range
        angle_range = max(abs(hip_max), abs(knee_max), 1.0)  # avoid /0
        total_time_sec = angle_range / speed
        n_points       = len(waypoints)
        dt             = total_time_sec / n_points
        dt             = max(dt, _SEND_PERIOD_SEC)   # floor at serial rate

        for wp in waypoints:

            loop_start = time.monotonic()

            # Stop check
            if self._stop_event.is_set():
                return False

            # Session continue check
            if not self._mgr.should_continue():
                return False

            # Pause: block here, keep sending current position
            if self._pause_event.is_set():
                while self._pause_event.is_set():
                    if self._stop_event.is_set():
                        return False
                    # Re-send current angles to hold position actively
                    current = self._serial.get_current_angles()
                    if not self._serial.is_halted():
                        self._serial.send_angles(current)
                    time.sleep(0.05)

            # Hardware halt check (E-stop from ESP32 or dashboard)
            if self._serial.is_halted():
                logger.warning(
                    "PassiveMode._execute_sweep: serial halted — waiting."
                )
                while self._serial.is_halted():
                    if self._stop_event.is_set():
                        return False
                    time.sleep(0.1)
                # After halt cleared, continue from current waypoint
                continue

            # Send this waypoint
            angles_list = wp.to_list()
            self._serial.send_angles(angles_list)

            # Log sample (20Hz sensor logging)
            self._log_sample(angles_list)

            # Sleep for remainder of dt
            elapsed    = time.monotonic() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        return True

    # =========================================================================
    #  Hold at position
    # =========================================================================

    def _interruptible_hold(self, duration_ms: int) -> bool:
        """
        Hold current position for duration_ms milliseconds.
        Sliced into 50ms intervals — interruptible by stop or pause-then-stop.

        While holding, re-sends current angles every 50ms to counteract
        any gravity-induced drift (important for backdrivable gearboxes).

        Args:
            duration_ms : Hold duration in milliseconds

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

            # Re-send current position every 50ms during hold
            if not self._serial.is_halted():
                current = self._serial.get_current_angles()
                self._serial.send_angles(current)

            time.sleep(0.05)

        return True

    # =========================================================================
    #  Return to home
    # =========================================================================

    def _return_to_home(self) -> None:
        """
        Smoothly move the arm back to home position (all axes at 0°).
        Called at the end of a completed or normally-stopped session.
        Uses half the normal trajectory points for a faster return.
        """
        logger.info("PassiveMode: returning to home.")

        current_list = self._serial.get_current_angles()
        current      = JointAngles.from_list(current_list)
        home         = JointAngles(ax1=0.0, ax2=0.0, ax3=0.0, ax4=0.0)

        # Skip if already at home
        if (abs(current.ax1) < 0.5 and abs(current.ax2) < 0.5
                and abs(current.ax3) < 0.5):
            logger.info("PassiveMode: already at home — skip return.")
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

        logger.info("PassiveMode: home reached.")

    # =========================================================================
    #  Sensor logging
    # =========================================================================

    def _log_sample(self, angles_list: List[float]) -> None:
        """
        Read current sensor state and buffer one sample in session_manager.
        Called on every waypoint — 20Hz.

        In passive mode:
            robot_effort_pct = 100.0  (robot does all the work)
            patient_effort_pct logged for record (co-contraction monitoring)
            intent_direction  logged for record (not used for control)

        Falls back to zeros if sensor_hub is None (dev mode).
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