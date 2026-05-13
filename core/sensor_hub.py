# =============================================================================
#  core/sensor_hub.py
#  /home/pi/rehab_robot/core/sensor_hub.py
#
#  Sensor Hub — FSR + sEMG Processing Layer
#
#  WHAT THIS MODULE DOES:
#    Sits on top of serial_comm.py and adds the intelligence layer:
#      1. Pulls raw FSR and EMG values from serial_comm at sample_rate_hz
#      2. Maintains a rolling window of EMG samples and computes RMS
#      3. Runs the bidirectional FSR intent detection:
#           PUSH  → fsr_raw > (fsr_rest_raw + fsr_push_threshold)
#           LIFT  → fsr_raw < (fsr_rest_raw - fsr_lift_threshold)
#           NONE  → fsr_raw within the neutral band
#      4. Runs the Truth-Check AND gate:
#           intent confirmed only if FSR intent AND EMG > threshold
#      5. Computes normalized force magnitude (0.0–1.0) for speed mapping
#      6. Computes patient effort % from EMG RMS for analytics dashboard
#      7. Applies debounce to prevent jitter/noise false triggers
#
#  INTENT DIRECTION EXPLAINED:
#    The single FSR is under the leg cuff.
#    - Leg rests naturally   → ADC ≈ fsr_rest_raw (neutral zone, no intent)
#    - Patient pushes DOWN   → ADC increases (flexion intent → PUSH)
#    - Patient lifts UP      → ADC decreases (extension intent → LIFT)
#    Both directions are valid therapy intents in active-assistive mode.
#    The direction tells the therapy engine which way to assist.
#
#  THREAD ARCHITECTURE:
#    _processing_thread : Runs at sample_rate_hz, pulls from serial_comm,
#                         processes all signals, updates shared state.
#    All public methods are read-only and thread-safe.
#    No writes to serial_comm happen here — sensor_hub only reads.
#
#  USAGE (already wired in main.py):
#    hub = SensorHub(serial_comm=comm)
#    hub.start()
#    hub.is_running()              → True/False
#    hub.get_intent()              → ('PUSH'|'LIFT'|'NONE', magnitude: float)
#    hub.get_emg_rms()             → float
#    hub.get_patient_effort_pct()  → float 0.0–100.0
#    hub.get_raw()                 → dict with fsr, emg, timestamp
#    hub.get_full_status()         → dict for dashboard/WebSocket
#    hub.stop()
# =============================================================================

import threading
import time
import math
import logging
from collections import deque
from enum import Enum, auto
from typing import Tuple, Optional

from core.config import CFG

logger = logging.getLogger(__name__)


# =============================================================================
#  Intent Direction
# =============================================================================

class IntentDirection(Enum):
    NONE = auto()   # No confirmed movement intent
    PUSH = auto()   # Patient pushing DOWN → flexion intent
    LIFT = auto()   # Patient lifting UP   → extension intent


# =============================================================================
#  SensorHub
# =============================================================================

class SensorHub:
    """
    Processes FSR and sEMG signals from serial_comm into clean, usable
    intent signals and effort metrics for the therapy engine and dashboard.
    """

    def __init__(self, serial_comm):
        """
        Args:
            serial_comm: Running SerialComm instance.
                         SensorHub reads from it but never writes to it.
        """
        self._serial_comm = serial_comm

        # ── Processing thread ──────────────────────────────────────────────
        self._thread     : Optional[threading.Thread] = None
        self._stop_event : threading.Event            = threading.Event()
        self._running    : bool                       = False

        # ── EMG rolling window for RMS ─────────────────────────────────────
        # deque with fixed maxlen automatically discards oldest samples
        self._emg_window : deque = deque(
            maxlen=CFG.sensors.emg_rms_window_samples
        )

        # ── Debounce state ─────────────────────────────────────────────────
        # Intent must be stable for intent_debounce_ms before it is confirmed.
        # Prevents a single noisy sample from triggering assist.
        self._debounce_ms         = CFG.active_mode.intent_debounce_ms
        self._raw_intent          = IntentDirection.NONE  # What sensor sees NOW
        self._raw_intent_since    = 0.0                   # time.monotonic()
        self._confirmed_intent    = IntentDirection.NONE  # Debounced output
        self._confirmed_magnitude = 0.0                   # 0.0–1.0

        # ── Shared output state ────────────────────────────────────────────
        # Written by _processing_thread, read by all public methods.
        # Protected by _state_lock.
        self._state_lock = threading.Lock()
        self._state = {
            # Raw sensor values (latest from serial_comm)
            "fsr_raw"           : 0.0,
            "emg_raw"           : 0.0,

            # Processed values
            "emg_rms"           : 0.0,    # RMS over rolling window
            "fsr_delta"         : 0.0,    # fsr_raw - fsr_rest_raw (signed)
            "fsr_normalized"    : 0.0,    # abs(fsr_delta) / fsr_max_delta_raw

            # Intent (after debounce + AND gate)
            "intent_direction"  : IntentDirection.NONE,
            "intent_magnitude"  : 0.0,    # 0.0–1.0 force strength

            # Individual gate states (useful for debugging and dashboard)
            "fsr_gate"          : False,  # FSR threshold exceeded
            "emg_gate"          : False,  # EMG threshold exceeded
            "intent_confirmed"  : False,  # Both gates passed + debounced

            # Analytics
            "patient_effort_pct": 0.0,    # 0.0–100.0 based on EMG RMS

            # Diagnostics
            "last_update"       : None,   # time.monotonic() of last processing cycle
            "sensor_age_ms"     : None,   # Age of latest sensor packet from Arduino
            "sensor_stale"      : False,  # True if no new packet for >200ms
            "total_cycles"      : 0,
        }

        # ── Stale sensor detection ─────────────────────────────────────────
        # If Arduino stops sending packets (serial error, disconnect),
        # flag it so therapy modes can pause safely.
        self._stale_threshold_ms = 2000   # ms without a new packet = stale

        logger.info(
            f"SensorHub created. "
            f"EMG RMS window={CFG.sensors.emg_rms_window_samples} samples "
            f"({CFG.sensors.rms_window_sec:.2f}s), "
            f"Debounce={self._debounce_ms}ms"
        )


    # =========================================================================
    #  Lifecycle
    # =========================================================================

    def start(self):
        """
        Starts the background processing thread.
        Raises RuntimeError if already running.
        """
        if self._running:
            logger.warning("SensorHub.start() called but already running.")
            return

        if not self._serial_comm.is_open():
            raise RuntimeError(
                "SensorHub.start() called but SerialComm port is not open. "
                "Open and handshake SerialComm before starting SensorHub."
            )

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._processing_loop,
            name="SensorHub-Processor",
            daemon=True
        )
        self._thread.start()
        self._running = True

        logger.info(
            f"SensorHub started. "
            f"Thread: {self._thread.name}, "
            f"Rate: {CFG.sensors.sample_rate_hz}Hz"
        )


    def stop(self):
        """
        Stops the processing thread cleanly.
        Called during system shutdown.
        """
        logger.info("SensorHub stopping...")
        self._stop_event.set()
        self._running = False

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                logger.warning("SensorHub thread did not stop within timeout.")

        logger.info("SensorHub stopped.")


    def is_running(self) -> bool:
        """Returns True if the processing thread is alive and running."""
        return (
            self._running and
            self._thread is not None and
            self._thread.is_alive()
        )


    # =========================================================================
    #  Public Interface — called by therapy modules and Flask routes
    # =========================================================================

    def get_intent(self) -> Tuple[str, float]:
        """
        Returns the current confirmed movement intent.

        Returns:
            Tuple of (direction: str, magnitude: float)

            direction:
                'PUSH' — patient pushing leg down (flexion intent)
                'LIFT' — patient lifting leg up (extension intent)
                'NONE' — no confirmed intent (neutral or below threshold)

            magnitude:
                0.0–1.0 — normalized force strength.
                0.0 = just at threshold.
                1.0 = maximum expected force (fsr_max_delta_raw).
                Used by active_assist.py to map to motor speed.
                Always 0.0 when direction is 'NONE'.

        Example usage in active_assist.py:
            direction, magnitude = sensor_hub.get_intent()
            if direction == 'PUSH':
                speed = map(magnitude, 0, 1, min_speed, max_speed)
                serial_comm.send_angles(flex_target)
            elif direction == 'LIFT':
                speed = map(magnitude, 0, 1, min_speed, max_speed)
                serial_comm.send_angles(extend_target)
        """
        with self._state_lock:
            direction = self._state["intent_direction"]
            magnitude = self._state["intent_magnitude"]
        return direction.name, magnitude


    def get_emg_rms(self) -> float:
        """
        Returns current EMG RMS value over the rolling window.
        Range: 0.0–1023.0 (raw ADC scale).
        Used by session_manager to log emg_rms to database.
        """
        with self._state_lock:
            return self._state["emg_rms"]


    def get_patient_effort_pct(self) -> float:
        """
        Returns patient effort as a percentage (0.0–100.0).

        Calculated as:
            (emg_rms / emg_max_rms) × 100
        Clamped to 0–100. Calibrate emg_max_rms in config.yaml by asking
        the patient to contract maximally and reading the peak RMS.

        Used by analytics.py to build the effort trend graph.
        """
        with self._state_lock:
            return self._state["patient_effort_pct"]


    def get_raw(self) -> dict:
        """
        Returns the latest raw sensor values from the Arduino.
        Used by the WebSocket streamer to feed live charts on the dashboard.

        Returns dict with keys:
            fsr_raw   : float — raw ADC 0–1023
            emg_raw   : float — raw ADC 0–1023
            fsr_delta : float — signed delta from rest baseline
                                positive = push, negative = lift
        """
        with self._state_lock:
            return {
                "fsr_raw"   : self._state["fsr_raw"],
                "emg_raw"   : self._state["emg_raw"],
                "fsr_delta" : self._state["fsr_delta"],
            }


    def is_sensor_stale(self) -> bool:
        """
        Returns True if no new sensor packet has arrived from Arduino
        for longer than the stale threshold (200ms).
        Therapy modes should pause or halt if this returns True.
        """
        with self._state_lock:
            return self._state["sensor_stale"]


    def get_full_status(self) -> dict:
        """
        Returns the complete sensor state as a dict.
        Used by Flask routes to serve the dashboard status endpoint
        and by the WebSocket streamer for live updates.

        Returns all keys from _state plus string version of intent_direction.
        """
        with self._state_lock:
            status = dict(self._state)

        # Convert enum to string for JSON serialisation
        status["intent_direction"] = status["intent_direction"].name

        # Add config thresholds so dashboard can show threshold lines on graphs
        status["fsr_rest_raw"]       = CFG.sensors.fsr_rest_raw
        status["fsr_push_threshold"] = CFG.sensors.fsr_push_threshold
        status["fsr_lift_threshold"] = CFG.sensors.fsr_lift_threshold
        status["emg_threshold"]      = CFG.sensors.emg_threshold

        return status


    def calibrate_fsr_rest(self) -> float:
        """
        Measures the current FSR rest baseline by averaging 50 samples
        over 500ms with the patient relaxed in the cuff.

        Call this at the start of each session before therapy begins.
        Updates CFG.sensors.fsr_rest_raw in memory (not saved to config file —
        the therapist should update config.yaml manually if needed).

        Returns:
            float — measured rest baseline ADC value
        """
        logger.info(
            "Calibrating FSR rest baseline. "
            "Ensure patient is relaxed in cuff with no intentional push or lift..."
        )

        samples = []
        sample_count = 50
        interval_sec = 0.01   # 100Hz

        for _ in range(sample_count):
            data = self._serial_comm.get_latest_sensors()
            if data["timestamp"] is not None:
                samples.append(data["fsr"])
            time.sleep(interval_sec)

        if not samples:
            logger.error("FSR calibration failed — no sensor data received.")
            return CFG.sensors.fsr_rest_raw

        rest_value = sum(samples) / len(samples)
        logger.info(
            f"FSR rest baseline calibrated: {rest_value:.1f} ADC "
            f"(was {CFG.sensors.fsr_rest_raw}). "
            f"Update fsr_rest_raw in config.yaml to persist."
        )

        # Update in memory so this session uses the fresh value
        CFG.sensors.fsr_rest_raw = rest_value
        return rest_value


    # =========================================================================
    #  Processing Loop
    # =========================================================================

    def _processing_loop(self):
        """
        Runs on SensorHub-Processor thread at sample_rate_hz.
        Reads from serial_comm, processes all signals, updates shared state.
        """
        logger.debug("SensorHub processing loop started.")

        # Processing interval in seconds
        interval_sec = 1.0 / CFG.sensors.sample_rate_hz

        # Track last serial packet timestamp to detect stale data
        last_packet_timestamp = None

        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            try:
                self._process_cycle(last_packet_timestamp)

                # Update last known packet timestamp
                data = self._serial_comm.get_latest_sensors()
                last_packet_timestamp = data.get("timestamp")

            except Exception as e:
                logger.error(
                    f"SensorHub processing error: {e}", exc_info=True
                )

            # Sleep for remainder of interval to maintain rate
            elapsed   = time.monotonic() - loop_start
            remaining = interval_sec - elapsed
            if remaining > 0:
                time.sleep(remaining)

        logger.debug("SensorHub processing loop exiting.")


    def _process_cycle(self, last_packet_timestamp):
        """
        One processing cycle — called once per sample period.
        Reads sensors, computes all derived values, updates state.
        """
        # ── 1. Read latest sensor data from serial_comm ────────────────────
        data = self._serial_comm.get_latest_sensors()

        fsr_raw = data["fsr"]
        emg_raw = data["emg"]
        packet_timestamp = data.get("timestamp")

        # ── 2. Detect stale sensor data ────────────────────────────────────
        sensor_stale = False
        sensor_age_ms = None

        if packet_timestamp is not None:
            sensor_age_ms = (time.monotonic() - packet_timestamp) * 1000
            sensor_stale  = sensor_age_ms > self._stale_threshold_ms
            if sensor_stale:
                logger.warning(
                    f"Sensor data stale: last packet {sensor_age_ms:.0f}ms ago "
                    f"(threshold={self._stale_threshold_ms}ms). "
                    f"Arduino may have stopped sending."
                )
        else:
            # No packet received yet since startup
            sensor_stale = True

        # ── 3. EMG RMS calculation ─────────────────────────────────────────
        self._emg_window.append(emg_raw)
        emg_rms = self._compute_rms(self._emg_window)

        # ── 4. Patient effort % ────────────────────────────────────────────
        # Normalized EMG RMS relative to max expected contraction.
        # Clamped to 0–100.
        emg_max = CFG.sensors.emg_max_rms
        patient_effort_pct = min(100.0, (emg_rms / emg_max) * 100.0) \
                             if emg_max > 0 else 0.0

        # ── 5. FSR bidirectional intent detection ──────────────────────────
        rest      = CFG.sensors.fsr_rest_raw
        fsr_delta = fsr_raw - rest   # Positive = push, Negative = lift

        push_band = CFG.sensors.fsr_push_threshold
        lift_band = CFG.sensors.fsr_lift_threshold
        max_delta = CFG.sensors.fsr_max_delta_raw

        if fsr_delta > push_band:
            # Patient is pushing DOWN — flexion intent
            raw_fsr_direction = IntentDirection.PUSH
            fsr_force_magnitude = min(1.0, fsr_delta / max_delta)
        elif fsr_delta < -lift_band:
            # Patient is lifting UP — extension intent
            raw_fsr_direction = IntentDirection.LIFT
            fsr_force_magnitude = min(1.0, abs(fsr_delta) / max_delta)
        else:
            # Within neutral band — no intent
            raw_fsr_direction   = IntentDirection.NONE
            fsr_force_magnitude = 0.0

        fsr_normalized = min(1.0, abs(fsr_delta) / max_delta) \
                         if max_delta > 0 else 0.0

        # ── 6. EMG gate ────────────────────────────────────────────────────
        emg_gate = emg_rms >= CFG.sensors.emg_threshold

        # ── 7. FSR gate ────────────────────────────────────────────────────
        fsr_gate = raw_fsr_direction != IntentDirection.NONE

        # ── 8. Truth-Check AND gate ────────────────────────────────────────
        # Both FSR and EMG must confirm intent.
        # If require_both_sensors is False, FSR alone is sufficient.
        if CFG.sensors.require_both_sensors:
            raw_intent_active = fsr_gate and emg_gate
        else:
            raw_intent_active = fsr_gate

        raw_intent = raw_fsr_direction if raw_intent_active \
                     else IntentDirection.NONE

        # ── 9. Debounce ────────────────────────────────────────────────────
        # Intent must be stable for debounce_ms before it is confirmed.
        # This prevents a momentary noise spike from triggering assist.
        now = time.monotonic()
        confirmed_intent    = IntentDirection.NONE
        confirmed_magnitude = 0.0

        if raw_intent == self._raw_intent and \
           raw_intent != IntentDirection.NONE:
            # Intent is holding steady — check if debounce period has passed
            stable_duration_ms = (now - self._raw_intent_since) * 1000
            if stable_duration_ms >= self._debounce_ms:
                confirmed_intent    = raw_intent
                confirmed_magnitude = fsr_force_magnitude
        else:
            # Intent changed (or went to NONE) — reset debounce timer
            if raw_intent != self._raw_intent:
                self._raw_intent       = raw_intent
                self._raw_intent_since = now

        self._confirmed_intent    = confirmed_intent
        self._confirmed_magnitude = confirmed_magnitude

        # ── 10. Update shared state ────────────────────────────────────────
        with self._state_lock:
            self._state["fsr_raw"]            = fsr_raw
            self._state["emg_raw"]            = emg_raw
            self._state["emg_rms"]            = emg_rms
            self._state["fsr_delta"]          = fsr_delta
            self._state["fsr_normalized"]     = fsr_normalized
            self._state["intent_direction"]   = confirmed_intent
            self._state["intent_magnitude"]   = confirmed_magnitude
            self._state["fsr_gate"]           = fsr_gate
            self._state["emg_gate"]           = emg_gate
            self._state["intent_confirmed"]   = confirmed_intent != IntentDirection.NONE
            self._state["patient_effort_pct"] = patient_effort_pct
            self._state["last_update"]        = now
            self._state["sensor_age_ms"]      = sensor_age_ms
            self._state["sensor_stale"]       = sensor_stale
            self._state["total_cycles"]      += 1

        # ── 11. Debug logging (only at DEBUG level — not spam at INFO) ─────
        if confirmed_intent != IntentDirection.NONE:
            logger.debug(
                f"Intent confirmed: {confirmed_intent.name} | "
                f"Magnitude: {confirmed_magnitude:.2f} | "
                f"FSR delta: {fsr_delta:+.0f} | "
                f"EMG RMS: {emg_rms:.1f}"
            )


    # =========================================================================
    #  Signal Processing
    # =========================================================================

    @staticmethod
    def _compute_rms(samples: deque) -> float:
        """
        Computes Root Mean Square of a deque of samples.

        RMS = sqrt( mean( x_i^2 ) )

        This is the standard method for quantifying sEMG signal amplitude.
        Returns 0.0 if the deque is empty.
        """
        if not samples:
            return 0.0
        sum_squares = sum(x * x for x in samples)
        return math.sqrt(sum_squares / len(samples))
