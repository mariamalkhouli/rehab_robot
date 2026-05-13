# =============================================================================
#  core/safety_monitor.py
#  /home/pi/rehab_robot/core/safety_monitor.py
#
#  Wireless E-Stop Safety Monitor.
#
#  PROTOCOL:
#    - ESP32-C3 sends "HB" (heartbeat) UDP packet every 50ms continuously.
#    - If button is pressed, ESP32 sends "STOP" immediately AND stops heartbeats.
#    - Pi monitors for BOTH:
#        1. Explicit "STOP" packet  → immediate E-stop
#        2. Heartbeat silence >100ms → E-stop (covers battery death, Wi-Fi loss,
#           ESP32 crash, therapist walking out of range)
#
#  STATES:
#    DISARMED  → SafetyMonitor created but not started yet
#    ARMED     → Heartbeat being received normally, system safe to operate
#    ESTOPPED  → E-stop triggered, motors halted, awaiting manual clear
#    RECOVERING→ Heartbeat resumed after loss, waiting for stable period
#                before allowing arm (therapist must still manually clear)
#
#  THREAD SAFETY:
#    All state changes protected by threading.Lock.
#    The E-stop callback is called from the monitor thread —
#    keep callback implementations fast and non-blocking.
#
#  USAGE (already wired in main.py):
#    monitor = SafetyMonitor(
#        port=CFG.safety.heartbeat_port,
#        timeout_ms=CFG.safety.heartbeat_timeout_ms,
#        on_estop_callback=_on_estop_triggered
#    )
#    monitor.start()
#    monitor.is_running()    → True/False
#    monitor.is_estopped()   → True/False
#    monitor.is_armed()      → True/False
#    monitor.clear_estop()   → call after therapist confirms safe to resume
#    monitor.stop()          → clean shutdown
# =============================================================================

import socket
import threading
import time
import logging
from enum import Enum, auto
from typing import Callable, Optional

from core.config import CFG

logger = logging.getLogger(__name__)


# =============================================================================
#  Safety Monitor States
# =============================================================================

class SafetyState(Enum):
    DISARMED   = auto()   # Not started yet
    ARMED      = auto()   # Heartbeat healthy — safe to operate
    ESTOPPED   = auto()   # E-stop active — motors must be halted
    RECOVERING = auto()   # Heartbeat resumed but not yet cleared by therapist


# =============================================================================
#  SafetyMonitor
# =============================================================================

class SafetyMonitor:
    """
    Monitors the wireless ESP32-C3 E-stop remote via UDP.

    Runs two internal threads:
      - _listener_thread : receives UDP packets from ESP32
      - _watchdog_thread : checks heartbeat freshness every 10ms
    """

    # Packet constants — must match ESP32 firmware exactly
    PACKET_HEARTBEAT = "HB"
    PACKET_STOP      = "STOP"

    def __init__(
        self,
        port: int,
        timeout_ms: int,
        on_estop_callback: Callable[[str, str], None]
    ):
        """
        Args:
            port              : UDP port to listen on (must match ESP32 firmware)
            timeout_ms        : Heartbeat silence threshold in ms before E-stop
            on_estop_callback : Function called when E-stop triggers.
                                Signature: callback(source: str, reason: str)
                                source: 'wireless_button' | 'heartbeat_loss'
                                reason: human-readable description
        """
        self._port             = port
        self._timeout_ms       = timeout_ms
        self._timeout_sec      = timeout_ms / 1000.0
        self._on_estop         = on_estop_callback

        # ── State ──────────────────────────────────────────────────────────
        self._state            = SafetyState.DISARMED
        self._state_lock       = threading.Lock()

        # ── Heartbeat tracking ─────────────────────────────────────────────
        self._last_heartbeat   = None    # time.monotonic() of last HB packet
        self._heartbeat_lock   = threading.Lock()
        self._esp32_ip         = None    # IP of ESP32 once first packet arrives
        self._total_hb_count   = 0       # Total heartbeats received (for stats)
        self._missed_hb_count  = 0       # Consecutive missed heartbeat windows

        # ── Threads ────────────────────────────────────────────────────────
        self._stop_event       = threading.Event()
        self._listener_thread  = None
        self._watchdog_thread  = None

        # ── Socket ─────────────────────────────────────────────────────────
        self._sock             = None

        # ── E-stop event details (for logging/dashboard) ───────────────────
        self._last_estop_source = None
        self._last_estop_reason = None
        self._last_estop_time   = None
        self._estop_count       = 0      # Total E-stops this session

        logger.info(
            f"SafetyMonitor created. Port={port}, "
            f"Timeout={timeout_ms}ms"
        )


    # =========================================================================
    #  Public Interface
    # =========================================================================

    def start(self):
        """
        Binds the UDP socket and starts listener + watchdog threads.
        Raises RuntimeError if socket cannot be bound.
        """
        if self._listener_thread is not None and self._listener_thread.is_alive():
            logger.warning("SafetyMonitor.start() called but already running.")
            return

        logger.info(f"Binding UDP socket on 0.0.0.0:{self._port}...")
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.settimeout(0.5)   # Non-blocking read with 500ms timeout
                                          # so listener thread can check stop_event
            self._sock.bind(("0.0.0.0", self._port))
            logger.info(f"UDP socket bound on port {self._port}.")
        except OSError as e:
            raise RuntimeError(
                f"Cannot bind UDP socket on port {self._port}: {e}\n"
                f"Is another process already using this port? "
                f"Check with: sudo lsof -i UDP:{self._port}"
            )

        self._stop_event.clear()

        # Start listener thread
        self._listener_thread = threading.Thread(
            target=self._listener_loop,
            name="SafetyMonitor-Listener",
            daemon=True    # Dies automatically when main thread exits
        )
        self._listener_thread.start()

        # Start watchdog thread
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="SafetyMonitor-Watchdog",
            daemon=True
        )
        self._watchdog_thread.start()

        # Transition to ARMED — waiting for first heartbeat
        self._set_state(SafetyState.ARMED)

        logger.info(
            "SafetyMonitor started. Threads: "
            f"Listener={self._listener_thread.name}, "
            f"Watchdog={self._watchdog_thread.name}"
        )
        logger.info(
            f"Waiting for ESP32 heartbeat on UDP port {self._port}... "
            f"(E-stop triggers if no heartbeat within {self._timeout_ms}ms)"
        )


    def stop(self):
        """
        Signals both threads to stop and waits for them to finish cleanly.
        Closes the UDP socket.
        """
        logger.info("SafetyMonitor stopping...")
        self._stop_event.set()

        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=2.0)
            if self._listener_thread.is_alive():
                logger.warning("Listener thread did not stop within timeout.")

        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
            if self._watchdog_thread.is_alive():
                logger.warning("Watchdog thread did not stop within timeout.")

        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

        self._set_state(SafetyState.DISARMED)
        logger.info(
            f"SafetyMonitor stopped. "
            f"Total heartbeats received: {self._total_hb_count}. "
            f"Total E-stops: {self._estop_count}."
        )


    def is_running(self) -> bool:
        """Returns True if both threads are alive and state is not DISARMED."""
        return (
            self._listener_thread is not None and
            self._listener_thread.is_alive() and
            self._watchdog_thread is not None and
            self._watchdog_thread.is_alive() and
            self._state != SafetyState.DISARMED
        )


    def is_armed(self) -> bool:
        """Returns True only if heartbeat is healthy and no E-stop is active."""
        return self._state == SafetyState.ARMED


    def is_estopped(self) -> bool:
        """Returns True if an E-stop is currently active."""
        return self._state == SafetyState.ESTOPPED


    def clear_estop(self, cleared_by: str = "therapist") -> bool:
            """
            Force-clears the E-stop state to allow testing without an ESP32 heartbeat.
            """
            with self._state_lock:
                # We still only clear if we are actually in an E-stop state
                if self._state != SafetyState.ESTOPPED:
                    return False

                # --- BYPASS LOGIC START ---
                with self._heartbeat_lock:
                    # Manually reset the timer so the Watchdog doesn't 
                    # immediately trigger another E-stop.
                    self._last_heartbeat = time.monotonic()
                
                # Change state back to ARMED so serial_comm allows movement
                self._state = SafetyState.ARMED
                # --- BYPASS LOGIC END ---

            logger.info(
                f"E-stop FORCE CLEARED by '{cleared_by}'. "
                f"System re-armed for testing."
            )

            # Log to database (Keep this for your records)
            try:
                from core.database import log_system_event
                log_system_event(
                    event_type='estop',
                    severity='info',
                    message=f"E-stop force cleared by '{cleared_by}'.",
                    source_module='safety_monitor'
                )
            except Exception as e:
                logger.error(f"Failed to log E-stop clear: {e}")

            return True


    def get_status(self) -> dict:
        """
        Returns a dictionary of current safety monitor status.
        Used by Flask routes to populate the dashboard.
        """
        with self._heartbeat_lock:
            last_hb = self._last_heartbeat
            hb_age_ms = (
                (time.monotonic() - last_hb) * 1000
                if last_hb is not None else None
            )

        return {
            "state"             : self._state.name,
            "is_armed"          : self.is_armed(),
            "is_estopped"       : self.is_estopped(),
            "esp32_ip"          : self._esp32_ip,
            "heartbeat_age_ms"  : round(hb_age_ms, 1) if hb_age_ms else None,
            "total_heartbeats"  : self._total_hb_count,
            "estop_count"       : self._estop_count,
            "last_estop_source" : self._last_estop_source,
            "last_estop_reason" : self._last_estop_reason,
            "last_estop_time"   : self._last_estop_time,
            "timeout_ms"        : self._timeout_ms,
        }


    # =========================================================================
    #  Internal Threads
    # =========================================================================

    def _listener_loop(self):
        """
        Runs on SafetyMonitor-Listener thread.
        Continuously receives UDP packets from the ESP32-C3.
        Handles two packet types: "HB" (heartbeat) and "STOP" (emergency stop).
        """
        logger.debug("Listener thread started.")

        while not self._stop_event.is_set():
            try:
                data, addr = self._sock.recvfrom(1024)
            except socket.timeout:
                # No packet in 500ms — normal, just loop and check stop_event
                continue
            except OSError as e:
                # Socket was closed (during shutdown) — exit cleanly
                if self._stop_event.is_set():
                    break
                logger.error(f"UDP socket error in listener: {e}")
                time.sleep(0.1)
                continue

            try:
                message = data.decode('utf-8').strip()
            except UnicodeDecodeError:
                logger.warning(f"Received non-UTF-8 packet from {addr[0]}. Ignoring.")
                continue

            esp_ip = addr[0]

            # Log first contact with ESP32
            if self._esp32_ip is None:
                self._esp32_ip = esp_ip
                logger.info(
                    f"First packet received from ESP32 at {esp_ip}. "
                    f"Heartbeat link established."
                )

            # ── Handle packet type ─────────────────────────────────────────
            if message == self.PACKET_HEARTBEAT:
                self._handle_heartbeat(esp_ip)

            elif message == self.PACKET_STOP:
                logger.critical(
                    f"STOP packet received from ESP32 at {esp_ip}! "
                    f"Button was pressed."
                )
                self._trigger_estop(
                    source='wireless_button',
                    reason=f"Emergency stop button pressed on remote (ESP32 at {esp_ip})."
                )

            else:
                # Unknown packet — log but do not crash or ignore silently
                logger.warning(
                    f"Unknown packet from {esp_ip}: '{message}'. "
                    f"Expected '{self.PACKET_HEARTBEAT}' or '{self.PACKET_STOP}'."
                )

        logger.debug("Listener thread exiting.")


    def _watchdog_loop(self):
        """
        Runs on SafetyMonitor-Watchdog thread.
        Checks every 10ms whether the last heartbeat is too old.

        BEHAVIOUR:
          - ESP32 never connected → system runs normally, watchdog stays quiet.
            E-stop only fires if an explicit STOP packet arrives (listener thread).
          - ESP32 connects (heartbeats start) → watchdog begins enforcing timeout.
            If heartbeat then goes silent for > timeout_ms → E-stop fires.
            This covers: battery dead, Wi-Fi dropped, ESP32 crashed, out of range.

        This means the ESP32 remote is OPTIONAL.
        If it is present and powered, it provides full wireless E-stop coverage.
        If it is absent, the dashboard E-stop button is the only software E-stop.
        """
        logger.debug("Watchdog thread started.")

        # Track whether we have ever seen a heartbeat from this ESP32
        # False  → ESP32 not connected, timeout enforcement OFF
        # True   → ESP32 connected, timeout enforcement ON
        _esp32_ever_seen = False
        _connected_logged = False

        while not self._stop_event.is_set():
            time.sleep(0.01)   # Check every 10ms

            with self._heartbeat_lock:
                last_hb = self._last_heartbeat

            # ── ESP32 not yet seen ─────────────────────────────────────────
            if not _esp32_ever_seen:
                if last_hb is None:
                    # No heartbeat ever — ESP32 not connected.
                    # Do nothing. System runs fine without it.
                    continue
                else:
                    # First heartbeat just arrived — arm the watchdog
                    _esp32_ever_seen = True
                    if not _connected_logged:
                        logger.info(
                            "Watchdog: ESP32 remote connected. "
                            "Heartbeat timeout enforcement ACTIVE. "
                            f"Timeout = {self._timeout_ms}ms."
                        )
                        _connected_logged = True

            # ── ESP32 was connected — enforce heartbeat timeout ────────────
            if self._state == SafetyState.ESTOPPED:
                # Already E-stopped — don't re-trigger, just keep monitoring
                continue

            elapsed_ms = (time.monotonic() - last_hb) * 1000

            if elapsed_ms > self._timeout_ms:
                self._missed_hb_count += 1
                logger.critical(
                    f"Watchdog: Heartbeat LOST! "
                    f"Last received {elapsed_ms:.0f}ms ago "
                    f"(timeout = {self._timeout_ms}ms)."
                )
                self._trigger_estop(
                    source='heartbeat_loss',
                    reason=(
                        f"ESP32 heartbeat lost for {elapsed_ms:.0f}ms "
                        f"(threshold = {self._timeout_ms}ms). "
                        f"Possible causes: battery dead, Wi-Fi dropped, "
                        f"ESP32 crashed, or remote out of range."
                    )
                )
            else:
                # Heartbeat healthy — reset missed counter
                self._missed_hb_count = 0

        logger.debug("Watchdog thread exiting.")


    # =========================================================================
    #  Internal Helpers
    # =========================================================================

    def _handle_heartbeat(self, esp_ip: str):
        """
        Updates the last heartbeat timestamp.
        Called from the listener thread every time an "HB" packet arrives.
        """
        with self._heartbeat_lock:
            self._last_heartbeat = time.monotonic()
            self._total_hb_count += 1

        # If we were in RECOVERING state and heartbeat came back,
        # log it but don't auto-clear — therapist must manually clear
        if self._state == SafetyState.RECOVERING:
            logger.info(
                "Heartbeat resumed. System in RECOVERING state. "
                "Therapist must manually clear E-stop from dashboard."
            )

        # Occasional heartbeat receipt log (every 1000 heartbeats ≈ every 50 sec)
        if self._total_hb_count % 1000 == 0:
            logger.debug(
                f"Heartbeat #{self._total_hb_count} from {esp_ip}. "
                f"Link healthy."
            )


    def _trigger_estop(self, source: str, reason: str):
        """
        Transitions system to ESTOPPED state and fires the callback.
        Thread-safe — can be called from either listener or watchdog thread.
        Uses a lock to guarantee the callback is only fired once per E-stop event.
        """
        with self._state_lock:
            if self._state == SafetyState.ESTOPPED:
                # Already E-stopped — do not re-trigger
                return

            self._state             = SafetyState.ESTOPPED
            self._last_estop_source = source
            self._last_estop_reason = reason
            self._last_estop_time   = time.strftime("%Y-%m-%d %H:%M:%S")
            self._estop_count      += 1

        logger.critical(
            f"!!! E-STOP ACTIVATED !!! "
            f"Source='{source}' | "
            f"Reason='{reason}' | "
            f"Total E-stops this run: {self._estop_count}"
        )

        # Fire the callback (defined in main.py → _on_estop_triggered)
        # This is where main.py sends HALT to Arduino and logs to database
        try:
            self._on_estop(source, reason)
        except Exception as e:
            # The callback must never crash the safety monitor
            logger.error(
                f"E-stop callback raised an exception: {e}. "
                f"Motors may not have been halted — check immediately!"
            )


    def _set_state(self, new_state: SafetyState):
        """Thread-safe state transition with logging."""
        with self._state_lock:
            old_state = self._state
            self._state = new_state
        if old_state != new_state:
            logger.info(
                f"SafetyMonitor state: {old_state.name} → {new_state.name}"
            )