# =============================================================================
#  therapy/active_assist.py
#  /home/rehabrobot/rehab_robot/therapy/active_assist.py
#
#  Active-Assistive Mode — Admittance Control
#
#  WHAT IT DOES:
#    Detects the patient's movement intent through FSR + EMG sensors and
#    assists the limb in the intended direction. The robot only moves when
#    the patient tries to move. The harder the patient pushes, the faster
#    the robot assists. When the patient stops trying, the robot holds.
#
#    This is the most clinically important mode for stroke rehabilitation.
#    It enforces active participation — the patient must initiate every
#    movement. This drives neuroplasticity far better than passive CPM.
#
#  ALGORITHM (one control loop at 20Hz):
#    1. Read intent: sensor_hub.get_intent() → (direction, magnitude)
#    2. NONE   → hold position, ramp speed down
#    3. PUSH   → step hip and knee toward max flex at mapped speed
#    4. LIFT   → step hip and knee toward 0° at mapped speed
#    5. Apply dynamic assistance scaling (reduces robot help as EMG rises)
#    6. Send via serial_comm.send_angles()
#    7. Update rep state machine
#    8. Log sample via session_manager.log_sample()
#    9. Sleep remainder of control period
#
#  SPEED MAPPING:
#    assist_speed = min_speed + magnitude × (max_speed - min_speed)
#    Config:  min_assist_speed_deg_per_sec = 2.0
#             max_assist_speed_deg_per_sec = 20.0
#    magnitude=0.0 → 2°/s,  magnitude=1.0 → 20°/s
#
#  DYNAMIC ASSISTANCE SCALING (assist-as-needed):
#    assist_scale = 1.0 - (patient_effort_pct/100 × 0.5)
#    final_speed  = mapped_speed × assist_scale
#    At 80% patient effort: scale = 1.0 - (0.8 × 0.5) = 0.60 → 60% of speed
#    At  0% patient effort: scale = 1.0                        → 100% of speed
#
#  SPEED RAMPING:
#    Ramp UP   over assist_ramp_up_ms   = 200ms when intent starts
#    Ramp DOWN over assist_ramp_down_ms = 300ms when intent stops
#    Prevents sudden jolts that trigger spastic catch.
#
#  REP COUNTING STATE MACHINE:
#    IDLE → FLEXING  : direction == 'PUSH'
#    FLEXING → PEAKED: ax2 >= hip_max×0.8 AND ax3 >= knee_max×0.8
#    PEAKED → EXTENDING: direction == 'LIFT'
#    EXTENDING → IDLE: ax2 <= 10° AND ax3 <= 10° → increment_reps()
#
#  CONFIG KEYS USED (exact names from config.yaml):
#    CFG.active_mode.control_loop_hz
#    CFG.active_mode.min_assist_speed_deg_per_sec
#    CFG.active_mode.max_assist_speed_deg_per_sec
#    CFG.active_mode.position_hold_tolerance_deg
#    CFG.active_mode.assist_ramp_up_ms
#    CFG.active_mode.assist_ramp_down_ms
#    CFG.joints.hip_flex_ext.safe_min_deg
#    CFG.joints.hip_flex_ext.safe_max_deg
#    CFG.joints.knee_flex_ext.safe_min_deg
#    CFG.joints.knee_flex_ext.safe_max_deg
#    CFG.passive_mode.trajectory_points   (used for return-to-home)
# =============================================================================

import time
import threading
import logging
from enum import Enum, auto
from typing import List, Tuple

from core.config import CFG
from therapy.kinematics import get_engine, JointAngles

logger = logging.getLogger(__name__)

# Control period — seconds per iteration
_CTRL_PERIOD_SEC = 1.0 / CFG.active_mode.control_loop_hz   # 0.05s at 20Hz

# Dynamic assistance reduction factor.
# At 100% patient effort, robot speed is reduced by this fraction.
# 0.5 → at full patient effort, robot moves at 50% of computed speed.
_ASSIST_REDUCTION_FACTOR = 0.5


# =============================================================================
#  Rep counting state machine
# =============================================================================

class _RepState(Enum):
    """
    Tracks patient progress through one complete flex-then-extend cycle.
    One increment_reps() call = patient completes full cycle.
    """
    IDLE      = auto()  # At or near home, waiting for PUSH
    FLEXING   = auto()  # Patient pushing toward max flex
    PEAKED    = auto()  # Reached >= 80% target — waiting for LIFT
    EXTENDING = auto()  # Patient returning toward home


# =============================================================================
#  ActiveAssist
# =============================================================================

class ActiveAssist:
    """
    Active-Assistive therapy mode — admittance control.

    Instantiated by SessionManager._launch_mode() when mode == 'active'.
    Runs on daemon thread 'ActiveAssist'.
    SessionManager calls stop() on session end or E-stop.

    Args:
        serial_comm     : core.serial_comm.SerialComm instance
        sensor_hub      : core.sensor_hub.SensorHub instance (None in dev mode)
        session_manager : therapy.session_manager.SessionManager instance
    """

    def __init__(self, serial_comm, sensor_hub, session_manager):
        self._serial = serial_comm
        self._sensor = sensor_hub
        self._mgr    = session_manager
        self._engine = get_engine()

        # Threading controls
        self._stop_event  = threading.Event()
        self._pause_event = threading.Event()

        # Worker thread
        self._thread = threading.Thread(
            target = self._run,
            name   = "ActiveAssist",
            daemon = True,
        )

        # Speed ramp state — tracks current smooth speed value
        self._current_speed : float   = 0.0

        # Rep state machine
        self._rep_state : _RepState = _RepState.IDLE

        logger.info("ActiveAssist: initialised.")

    # =========================================================================
    #  Public interface — called from SessionManager (Flask HTTP thread)
    # =========================================================================

    def start(self) -> None:
        """Launch the control thread. Called once by SessionManager."""
        logger.info("ActiveAssist: starting control thread.")
        self._thread.start()

    def pause(self) -> None:
        """
        Pause assist. Robot re-sends current angles to hold position.
        Ramp state is preserved — resume continues from current speed.
        """
        logger.info("ActiveAssist: paused.")
        self._pause_event.set()

    def resume(self) -> None:
        """
        Resume assist after pause. Speed ramp resets to 0 so restart is smooth.
        """
        logger.info("ActiveAssist: resumed.")
        self._current_speed = 0.0
        self._pause_event.clear()

    def stop(self) -> None:
        """
        Signal thread to stop and block until it exits (max 5s).
        Called by SessionManager.stop_session() or handle_estop().
        Thread-safe — can be called from any thread.
        """
        logger.info("ActiveAssist: stop requested.")
        self._stop_event.set()
        self._pause_event.clear()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            logger.warning("ActiveAssist: thread did not exit within 5s.")
        else:
            logger.info("ActiveAssist: thread stopped cleanly.")

    # =========================================================================
    #  Main control loop — runs on ActiveAssist thread
    # =========================================================================

    def _run(self) -> None:
        """
        Admittance control loop running at control_loop_hz (20Hz).

        Each iteration is exactly _CTRL_PERIOD_SEC (0.05s).
        The loop reads intent, computes the next position step,
        sends it to the Arduino, then sleeps for the remaining time.
        """
        logger.info("ActiveAssist: control loop started.")

        # Configure kinematics engine for correct leg side
        self._engine.set_affected_side(self._mgr._leg)

        while True:
            loop_start = time.monotonic()

            # ── Guards ─────────────────────────────────────────────────────

            if self._stop_event.is_set():
                logger.info("ActiveAssist: stop event — exiting loop.")
                break

            if not self._mgr.should_continue():
                logger.info("ActiveAssist: session_manager says stop.")
                break

            if self._mgr.is_reps_target_reached():
                logger.info("ActiveAssist: reps target reached — ending session.")
                self._mgr.stop_session(status='completed')
                return

            # ── Pause ──────────────────────────────────────────────────────
            if self._pause_event.is_set():
                self._hold_position()
                self._sleep_remainder(loop_start)
                continue

            # ── Hardware halt (E-stop) ──────────────────────────────────────
            if self._serial.is_halted():
                logger.warning("ActiveAssist: serial halted — waiting.")
                while self._serial.is_halted():
                    if self._stop_event.is_set():
                        return
                    time.sleep(0.1)
                # Re-arm: reset speed ramp for smooth restart
                self._current_speed = 0.0
                continue

            # ── Stale sensor guard ──────────────────────────────────────────
            # Active mode REQUIRES live sensor data for safe operation.
            # If sensor data is stale (Arduino stopped sending), hold position.
            if self._sensor is not None and self._sensor.is_sensor_stale():
                logger.warning(
                    "ActiveAssist: sensor data stale — holding position."
                )
                self._hold_position()
                self._sleep_remainder(loop_start)
                continue

            # ── Read live ROM targets ───────────────────────────────────────
            # Re-read each iteration so therapist live adjustments
            # take effect immediately (not just at rep boundaries).
            rom      = self._mgr.get_rom()
            hip_max  = rom['hip_flex_max']    # degrees
            knee_max = rom['knee_flex_max']   # degrees

            # ── Read intent from sensor_hub ────────────────────────────────
            # direction : 'PUSH' | 'LIFT' | 'NONE'  (string — enum.name)
            # magnitude : 0.0–1.0 normalised FSR force
            direction, magnitude = self._get_intent()

            # ── Read current motor positions ────────────────────────────────
            # Returns [ax1, ax2, ax3, ax4] in degrees.
            # ax4 is not used for control — it is always recomputed by fill_m4.
            current_list = self._serial.get_current_angles()
            ax1_cur = current_list[0]   # Hip Ab/Ad — held constant
            ax2_cur = current_list[1]   # Hip Flex/Ext — primary control axis
            ax3_cur = current_list[2]   # Knee Flex/Ext — primary control axis

            # ── Read patient effort for scaling and logging ─────────────────
            patient_effort_pct = self._get_patient_effort()

            # ── Compute next position ───────────────────────────────────────
            if direction == 'NONE':
                # ── No intent: hold position, ramp speed down ───────────────
                self._current_speed = self._ramp_speed(
                    current = self._current_speed,
                    target  = 0.0,
                    ramp_ms = CFG.active_mode.assist_ramp_down_ms,
                )
                self._hold_position()
                angles_to_send = current_list

            else:
                # ── Intent detected: compute assist step ────────────────────

                # Step 1: Map force magnitude → raw assist speed (linear)
                #   magnitude=0.0 → min_assist_speed_deg_per_sec = 2.0°/s
                #   magnitude=1.0 → max_assist_speed_deg_per_sec = 20.0°/s
                raw_speed = (
                    CFG.active_mode.min_assist_speed_deg_per_sec
                    + magnitude * (
                        CFG.active_mode.max_assist_speed_deg_per_sec
                        - CFG.active_mode.min_assist_speed_deg_per_sec
                    )
                )

                # Step 2: Dynamic assistance scaling (assist-as-needed)
                #   As patient works harder (EMG rises), reduce robot speed.
                #   scale=1.0 at 0% effort (full robot assist)
                #   scale=0.6 at 80% effort (robot reduced to 60%)
                #   scale=0.5 at 100% effort (robot at 50% minimum)
                assist_scale = 1.0 - (
                    (patient_effort_pct / 100.0) * _ASSIST_REDUCTION_FACTOR
                )
                assist_scale = max(0.1, assist_scale)  # floor at 10%

                target_speed = raw_speed * assist_scale

                # Step 3: Ramp current speed toward target
                #   This prevents sudden jolt at intent start
                self._current_speed = self._ramp_speed(
                    current = self._current_speed,
                    target  = target_speed,
                    ramp_ms = CFG.active_mode.assist_ramp_up_ms,
                )

                # Step 4: Convert speed to position step for this iteration
                #   step = speed (°/s) × period (s)
                step = self._current_speed * _CTRL_PERIOD_SEC

# --- STEP 5: COMPUTE NEW ANGLES BASED ON EXERCISE TYPE ---
                
                # Get the current exercise from the manager
                ex = getattr(self._mgr, '_exercise', 'hip_flex_ext')
                
                # Start with current angles as the baseline
                ax1_target = ax1_cur
                ax2_target = ax2_cur
                ax3_target = ax3_cur

                if direction == 'PUSH':
                    # --- INTENT: FLEXION / PUSH DOWN ---
                    if ex == "hip_ab_ad":
                        ax1_target = ax1_cur + step
                    elif ex == "knee_flex_ext":
                        ax3_target = ax3_cur + step
                        ax2_target = ax3_target * 0.2 # Coupled thigh lift
                    else: # hip_flex_ext
                        ax2_target = ax2_cur + step

                else: # direction == 'LIFT'
                    # --- INTENT: EXTENSION / LIFT UP ---
                    if ex == "hip_ab_ad":
                        ax1_target = ax1_cur - step
                    elif ex == "knee_flex_ext":
                        ax3_target = ax3_cur - step
                        ax2_target = ax3_target * 0.2 # Coupled thigh lower
                    else: # hip_flex_ext
                        ax2_target = ax2_cur - step

                # --- STEP 6: BUNDLE, AUTO-LEVEL M4, AND CLAMP ---
                # We create a JointAngles object. We leave ax4 as 0.0 because 
                # clamp_to_safe_limits() now calculates it automatically.
                target = JointAngles(
                    ax1 = ax1_target,
                    ax2 = ax2_target,
                    ax3 = ax3_target,
                    ax4 = 0.0 
                )
                
                # This call now:
                # 1. Runs fill_m4() to keep the cuff parallel to the shank
                # 2. Clamps the angles to the therapist's set ROM (from config)
                target = self._engine.clamp_to_safe_limits(target)
                
                # Final check: Ensure we don't exceed the LIVE ROM limits set on dashboard
                target.ax1 = max(-15.0, min(rom['ab_ad_max'], target.ax1))
                target.ax2 = max(0.0,    min(rom['hip_flex_max'], target.ax2))
                target.ax3 = max(0.0,    min(rom['knee_flex_max'], target.ax3))
                
                angles_to_send = target.to_list()

                # Step 7: Send to Arduino
                self._serial.send_angles(angles_to_send)

            # ── Update rep state machine ────────────────────────────────────
            self._update_rep_state(
                ax2_cur   = ax2_cur,
                ax3_cur   = ax3_cur,
                hip_max   = hip_max,
                knee_max  = knee_max,
                direction = direction,
            )

            # ── Log sample to session_manager buffer ────────────────────────
            self._log_sample(
                angles_list        = angles_to_send,
                direction          = direction,
                patient_effort_pct = patient_effort_pct,
            )

            # ── Sleep remaining control period ──────────────────────────────
            self._sleep_remainder(loop_start)

        # ── Smooth return to home on normal exit ────────────────────────────
        if not self._serial.is_halted():
            self._return_to_home()

        logger.info("ActiveAssist: control loop exited.")

    # =========================================================================
    #  Speed ramp
    # =========================================================================

    def _ramp_speed(
        self,
        current : float,
        target  : float,
        ramp_ms : float,
    ) -> float:
        """
        Step current speed toward target speed, limited by ramp rate.

        The ramp_ms parameter defines how long it takes to go from
        0 to max_assist_speed_deg_per_sec. We derive the maximum
        speed change allowed per control period and apply it.

        Args:
            current : Current speed value (deg/s)
            target  : Desired speed value (deg/s)
            ramp_ms : Time to traverse full speed range (ms)

        Returns:
            New speed value (deg/s), clamped 0..max_assist_speed
        """
        max_speed  = CFG.active_mode.max_assist_speed_deg_per_sec
        ramp_sec   = ramp_ms / 1000.0
        # Maximum change per control period
        max_change = (max_speed / ramp_sec) * _CTRL_PERIOD_SEC

        if target > current:
            new_speed = min(current + max_change, target)
        else:
            new_speed = max(current - max_change, target)

        return max(0.0, min(max_speed, new_speed))

    # =========================================================================
    #  Position hold
    # =========================================================================

    def _hold_position(self) -> None:
        """
        Re-send current motor angles to actively maintain position.

        Essential for backdrivable gearboxes — without active re-sends,
        gravity will pull the limb back during holds and pauses.
        Uses serial_comm.get_current_angles() which is the software-tracked
        position and is always accurate after homing.
        """
        if self._serial.is_halted():
            return
        current = self._serial.get_current_angles()
        self._serial.send_angles(current)

    # =========================================================================
    #  Rep state machine
    # =========================================================================

    def _update_rep_state(
        self,
        ax2_cur   : float,
        ax3_cur   : float,
        hip_max   : float,
        knee_max  : float,
        direction : str,
    ) -> None:
        """
        Track patient progress through one full flex-then-extend cycle.

        Thresholds:
            Peak  : patient must reach >= 80% of therapist-set ROM target
                    (e.g. if hip_max=90°, patient must reach 72°)
            Home  : patient must return to <= 10° on both axes

        The 80% peak threshold tolerates patients who cannot reach full ROM
        but are making genuine therapeutic effort.
        The 10° home threshold confirms full functional extension.
        """
        peak_hip  = hip_max  * 0.80
        peak_knee = knee_max * 0.80
        home_thr  = 10.0    # degrees from zero = "returned to home"

        if self._rep_state == _RepState.IDLE:
            if direction == 'PUSH':
                self._rep_state = _RepState.FLEXING
                logger.debug("RepState: IDLE → FLEXING")

        elif self._rep_state == _RepState.FLEXING:
            if ax2_cur >= peak_hip and ax3_cur >= peak_knee:
                self._rep_state = _RepState.PEAKED
                logger.debug(
                    f"RepState: FLEXING → PEAKED "
                    f"(hip={ax2_cur:.1f}/{peak_hip:.1f}°, "
                    f"knee={ax3_cur:.1f}/{peak_knee:.1f}°)"
                )

        elif self._rep_state == _RepState.PEAKED:
            if direction == 'LIFT':
                self._rep_state = _RepState.EXTENDING
                logger.debug("RepState: PEAKED → EXTENDING")

        elif self._rep_state == _RepState.EXTENDING:
            if ax2_cur <= home_thr and ax3_cur <= home_thr:
                reps = self._mgr.increment_reps()
                rom  = self._mgr.get_rom()
                logger.info(
                    f"ActiveAssist: rep {reps}/{rom['reps_target']} complete."
                )
                self._rep_state = _RepState.IDLE
                logger.debug("RepState: EXTENDING → IDLE")

    # =========================================================================
    #  Return to home
    # =========================================================================

    def _return_to_home(self) -> None:
        """
        Smoothly return the arm to home position (all axes at 0°).
        Called when the session ends normally.
        Uses cosine trajectory at half the normal point count for speed.
        """
        logger.info("ActiveAssist: returning to home.")

        current_list = self._serial.get_current_angles()
        current      = JointAngles.from_list(current_list)
        home         = JointAngles(ax1=0.0, ax2=0.0, ax3=0.0, ax4=0.0)

        # Skip if already essentially at home
        if abs(current.ax2) < 0.5 and abs(current.ax3) < 0.5:
            logger.info("ActiveAssist: already at home — skip return.")
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
            time.sleep(_CTRL_PERIOD_SEC)

        logger.info("ActiveAssist: home reached.")

    # =========================================================================
    #  Sensor reading helpers
    # =========================================================================

    def _get_intent(self) -> Tuple[str, float]:
        """
        Read confirmed movement intent from sensor_hub.get_intent().

        Returns:
            ('PUSH'|'LIFT'|'NONE', magnitude: float 0.0–1.0)

        sensor_hub.get_intent() returns (direction.name, magnitude) where
        direction.name is the string representation of IntentDirection enum.
        Falls back to ('NONE', 0.0) in dev mode (sensor_hub is None).
        """
        if self._sensor is None:
            return 'NONE', 0.0
        return self._sensor.get_intent()

    def _get_patient_effort(self) -> float:
        """
        Read patient effort percentage from sensor_hub.get_patient_effort_pct().
        Returns 0.0–100.0. Falls back to 0.0 in dev mode.
        """
        if self._sensor is None:
            return 0.0
        return self._sensor.get_patient_effort_pct()

    # =========================================================================
    #  Sample logging
    # =========================================================================

    def _log_sample(
        self,
        angles_list        : List[float],
        direction          : str,
        patient_effort_pct : float,
    ) -> None:
        """
        Buffer one sensor + motor sample in session_manager for DB write.
        Called every control loop iteration (20Hz).

        robot_effort_pct = 100.0 - patient_effort_pct
        (robot contributes exactly what the patient does not).

        Falls back to zero values in dev mode (sensor_hub is None).
        """
        if self._sensor is None:
            self._mgr.log_sample(
                angles             = angles_list,
                fsr_raw            = 0.0,
                emg_raw            = 0.0,
                emg_rms            = 0.0,
                patient_effort_pct = 0.0,
                robot_effort_pct   = 100.0,
                intent_direction   = direction,
            )
            return

        raw     = self._sensor.get_raw()
        emg_rms = self._sensor.get_emg_rms()

        self._mgr.log_sample(
            angles             = angles_list,
            fsr_raw            = raw['fsr_raw'],
            emg_raw            = raw['emg_raw'],
            emg_rms            = emg_rms,
            patient_effort_pct = patient_effort_pct,
            robot_effort_pct   = max(0.0, 100.0 - patient_effort_pct),
            intent_direction   = direction,
        )

    # =========================================================================
    #  Timing
    # =========================================================================

    def _sleep_remainder(self, loop_start: float) -> None:
        """
        Sleep for the remaining time in the current control period.
        Keeps the loop running at exactly control_loop_hz regardless of
        how long the computation and send took.

        Args:
            loop_start : time.monotonic() captured at top of this iteration
        """
        elapsed    = time.monotonic() - loop_start
        sleep_time = _CTRL_PERIOD_SEC - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)