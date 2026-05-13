# =============================================================================
#  core/serial_comm.py
#  /home/pi/rehab_robot/core/serial_comm.py
#
#  Serial Communication Bridge — Raspberry Pi <-> Arduino Mega 2560
#
#  RESPONSIBILITIES:
#    Sending (Pi → Mega):
#      - Angle packets  : "A:{ax1},{ax2},{ax3},{ax4}\n"
#      - Halt command   : "HALT\n"          ← E-stop, stops all motors immediately
#      - Jog command    : "JOG:{axis},{steps}\n"  ← mimicry mode manual movement
#      - Home command   : "HOME:{axis}\n"   ← trigger homing on one axis
#      - Home all       : "HOMEALL\n"       ← trigger full homing sequence
#      - Enable motors  : "ENABLE\n"        ← re-enable drivers after halt
#      - Ping           : "PING\n"          ← check Arduino is alive
#
#    Receiving (Mega → Pi):
#      - Sensor packet  : "S:{fsr},{emg},{ls0},{ls1},{ls2},{ls3},{ls4},{ls5},{ls6},{ls7}\n"
#                         2 sensor values + 8 limit switch values = 10 fields total
#      - Handshake ack  : "MEGA_READY\n"
#      - Ping response  : "PONG\n"
#      - Home complete  : "HOMED:{axis}\n"
#      - Home all done  : "HOMEDALL\n"
#      - Limit hit      : "LIMIT:{axis}:{min|max}\n"
#      - Error          : "ERR:{message}\n"
#
#  THREAD ARCHITECTURE:
#    _reader_thread : Runs continuously, reads lines from serial port,
#                     parses them, updates shared state dict.
#                     This is the ONLY thread that reads from the serial port.
#    All writes (send_angles, send_halt, etc.) happen on the calling thread
#    but are protected by _write_lock to prevent interleaving.
#
#  PACKET FORMAT DETAILS:
#    Angle packet values are floats with 2 decimal places.
#    All values are comma-separated with no spaces.
#    Every packet ends with \n (newline).
#    Arduino reads with Serial.readStringUntil('\n').
#
#  MOTOR POSITION TRACKING:
#    Since there are no encoders, the Pi tracks motor positions in software.
#    _current_angles[] is updated every time send_angles() is called.
#    This is the "software encoder" — it is only accurate if:
#      a) No steps are lost (motor doesn't stall)
#      b) Homing is performed on startup to establish a known reference
#    After homing, all angles are zeroed to their home positions.
#
#  SAFETY RULES ENFORCED HERE:
#    1. send_angles() clamps all values to safe joint limits before sending
#    2. send_angles() refuses to send if _halted is True
#    3. All limit switch states are monitored — any trigger fires callback
#    4. If serial port disconnects mid-session, fires on_disconnect_callback
# =============================================================================

import serial
import serial.tools.list_ports
import threading
import time
import logging
from typing import Callable, Optional, List

from core.config import CFG

logger = logging.getLogger(__name__)


# =============================================================================
#  Packet format constants
#  Defined once here — must match Arduino firmware exactly
# =============================================================================

CMD_ANGLES    = "A"        # Angle command prefix
CMD_HALT      = "HALT"     # Emergency halt — stops all motors
CMD_JOG       = "JOG"      # Jog one axis by N steps
CMD_HOME      = "HOME"     # Home one axis
CMD_HOMEALL   = "HOMEALL"  # Home all axes in sequence
CMD_ENABLE    = "ENABLE"   # Re-enable motor drivers after halt
CMD_PING      = "PING"     # Liveness check

RESP_SENSOR   = "S"        # Sensor data packet prefix
RESP_PONG     = "PONG"     # Response to PING
RESP_HOMED    = "HOMED"    # One axis homing complete
RESP_HOMEDALL = "HOMEDALL" # All axes homing complete
RESP_LIMIT    = "LIMIT"    # Limit switch triggered
RESP_ERROR    = "ERR"      # Arduino-side error

# Number of limit switches in sensor packet (must match Arduino firmware)
NUM_LIMIT_SWITCHES = 8

# Number of axes
NUM_AXES = 4

# Sensor packet field count: 1 FSR + 1 EMG + 8 limit switches = 10
SENSOR_PACKET_FIELDS = 10


# =============================================================================
#  SerialComm
# =============================================================================

class SerialComm:
    """
    Manages all serial communication between Raspberry Pi and Arduino Mega.

    Usage (already wired in main.py):
        comm = SerialComm(port, baud_rate, timeout_sec)
        comm.open()
        comm.handshake(...)
        comm.send_angles([ax1, ax2, ax3, ax4])
        comm.send_halt()
        comm.send_jog(axis=1, steps=100)
        angles = comm.get_current_angles()
        data   = comm.get_latest_sensors()
        comm.close()
    """

    def __init__(
        self,
        port: str,
        baud_rate: int,
        timeout_sec: float,
        on_limit_switch_callback:  Optional[Callable[[int, str], None]] = None,
        on_disconnect_callback:    Optional[Callable[[], None]]         = None,
        on_home_complete_callback: Optional[Callable[[int], None]]      = None,
        on_error_callback:         Optional[Callable[[str], None]]      = None
    ):
        """
        Args:
            port                     : Serial port e.g. '/dev/ttyUSB0'
            baud_rate                : Must match Arduino Serial.begin()
            timeout_sec              : Serial read timeout
            on_limit_switch_callback : Called when any limit switch triggers.
                                       Signature: callback(axis: int, direction: str)
                                       axis: 1-4, direction: 'min' | 'max'
            on_disconnect_callback   : Called if serial port disconnects unexpectedly
            on_home_complete_callback: Called when an axis finishes homing.
                                       Signature: callback(axis: int)
                                       axis: 0 = all axes homed
            on_error_callback        : Called when Arduino sends an ERR: packet.
                                       Signature: callback(message: str)
        """
        self._port        = port
        self._baud_rate   = baud_rate
        self._timeout_sec = timeout_sec

        # Callbacks
        self._on_limit_switch  = on_limit_switch_callback
        self._on_disconnect    = on_disconnect_callback
        self._on_home_complete = on_home_complete_callback
        self._on_error         = on_error_callback

        # Serial port object
        self._serial: Optional[serial.Serial] = None

        # ── Write lock ─────────────────────────────────────────────────────
        # Ensures only one thread writes to serial at a time.
        self._write_lock = threading.Lock()

        # ── Reader thread ──────────────────────────────────────────────────
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event    = threading.Event()

        # ── Shared sensor state ────────────────────────────────────────────
        self._sensor_lock = threading.Lock()
        self._latest_sensors: dict = {
            "fsr"           : 0.0,
            "emg"           : 0.0,
            "limit_switches": [0] * NUM_LIMIT_SWITCHES,
            "timestamp"     : None,
            "packet_count"  : 0,
        }

        # ── Motor position tracking (software encoder) ─────────────────────
        self._angles_lock    = threading.Lock()
        self._current_angles = [0.0] * NUM_AXES
        self._is_homed       = [False] * NUM_AXES

        # ── Homing state ───────────────────────────────────────────────────
        self._homing_event = threading.Event()

        # ── System state ───────────────────────────────────────────────────
        self._halted            = False
        self._is_open           = False
        self._active_session_id: Optional[int] = None

        # ── Rate limiting for send_angles ──────────────────────────────────
        # FIX: Ensure explicit calculation of send period (1.0 / 20Hz = 0.05s)
        self._send_period_sec = 1.0 / CFG.serial.send_rate_hz
        self._last_send_time  = 0.0

        # ── Limit switch edge detection ────────────────────────────────────
        self._prev_limit_states: List[int] = [0] * NUM_LIMIT_SWITCHES

        # ── Statistics ─────────────────────────────────────────────────────
        self._packets_sent     = 0
        self._packets_received = 0
        self._parse_errors     = 0
        self._limit_triggers   = 0

        logger.info(
            f"SerialComm created. Port={port}, "
            f"Baud={baud_rate}, Timeout={timeout_sec}s"
        )


    # =========================================================================
    #  Connection Management
    # =========================================================================

    def open(self):
        """
        Opens the serial port.
        Handshake and Threading are handled in handshake() to avoid race conditions.
        """
        if self._is_open:
            logger.warning("SerialComm.open() called but port is already open.")
            return

        logger.info(f"Opening serial port {self._port} at {self._baud_rate} baud...")

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud_rate,
                timeout=self._timeout_sec,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
        except serial.SerialException as e:
            raise serial.SerialException(
                f"Cannot open serial port '{self._port}': {e}\n"
                f"Available ports: {self._list_available_ports()}"
            )

        # Arduino resets when serial opens — wait for it to boot
        logger.info("Serial port hardware opened. Waiting for Arduino to boot (2s)...")
        time.sleep(2.0)

        # Flush any garbage from the boot reset
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

        self._is_open = True
        self._halted  = False

        logger.info("Serial port ready. Proceeding to handshake.")


    def handshake(
        self,
        send_token: str,
        expected_ack: str,
        retries: int,
        delay_sec: float
    ) -> bool:
        """
        Sends the handshake token to Arduino and waits for acknowledgement.
        Starts the reader thread ONLY after successful handshake.
        """
        if not self._is_open:
            raise RuntimeError("Cannot handshake: serial port is not open.")

        logger.info(
            f"Starting handshake. Sending '{send_token}', retries={retries}"
        )

        for attempt in range(1, retries + 1):
            # Clear buffers before each attempt
            self._serial.reset_input_buffer()

            # Send handshake token
            self._raw_write(f"{send_token}\n")

            # Wait for expected acknowledgement
            deadline = time.monotonic() + delay_sec
            while time.monotonic() < deadline:
                line = self._safe_readline()
                if line is None:
                    break
                if line == expected_ack:
                    logger.info(f"Handshake successful. Arduino responded: '{expected_ack}'")
                    
                    # ── START THE READER THREAD NOW ──────────────────────────────
                    # This prevents the reader thread from stealing the handshake response.
                    self._stop_event.clear()
                    self._reader_thread = threading.Thread(
                        target=self._reader_loop,
                        name="SerialComm-Reader",
                        daemon=True
                    )
                    self._reader_thread.start()
                    logger.info(f"Background reader thread started: {self._reader_thread.name}")
                    return True

            logger.warning(f"Handshake attempt {attempt} timed out. Retrying...")
            time.sleep(0.5)

        logger.error(f"Handshake FAILED after {retries} attempts.")
        return False


    def close(self):
        """
        Stops the reader thread and closes the serial port cleanly.
        """
        logger.info("SerialComm closing...")

        self._stop_event.set()

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)

        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
                logger.info("Serial port closed.")
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")

        self._is_open = False


    def is_open(self) -> bool:
        """Returns True if serial port is open."""
        return (self._is_open and self._serial and self._serial.is_open)


    # =========================================================================
    #  Sending Commands (Pi → Arduino)
    # =========================================================================

    def send_angles(self, angles: List[float]) -> bool:
        """
        Sends a 4-axis angle command to the Arduino.
        """
        if not self._is_open:
            return False

        if self._halted:
            logger.warning("send_angles: BLOCKED — System is halted. Clear E-Stop on Dashboard.")
            return False

        if len(angles) != NUM_AXES:
            return False

        # ── Rate limiting ──────────────────────────────────────────────────
        now = time.monotonic()
        if now - self._last_send_time < self._send_period_sec:
            return False

        # ── Clamp to safe joint limits ─────────────────────────────────────
        clamped = self._clamp_to_joint_limits(angles)

        # ── Build and send packet ──────────────────────────────────────────
        packet = (
            f"{CMD_ANGLES}:"
            f"{clamped[0]:.2f},"
            f"{clamped[1]:.2f},"
            f"{clamped[2]:.2f},"
            f"{clamped[3]:.2f}\n"
        )
        print(f"TX TO ARDUINO: {packet.strip()}") # Add this


        success = self._raw_write(packet)

        if success:
            with self._angles_lock:
                self._current_angles = list(clamped)
            self._last_send_time = now
            self._packets_sent  += 1

        return success


    def send_halt(self) -> bool:
        """Sends HALT command to Arduino — stops all motors immediately."""
        if not self._is_open: return False

        logger.critical("Sending HALT to Arduino Mega.")
        success = self._raw_write(f"{CMD_HALT}\n")

        if success:
            self._halted = True
        return success


    def send_jog(self, axis: int, steps: int) -> bool:
        """Sends a manual jog command."""
        if not self._is_open or self._halted: return False

        packet  = f"{CMD_JOG}:{axis},{steps}\n"
        success = self._raw_write(packet)

        if success:
            deg_change = steps / CFG.motors.computed_steps_per_deg
            with self._angles_lock:
                self._current_angles[axis - 1] += deg_change
                self._current_angles = list(self._clamp_to_joint_limits(self._current_angles))
            self._packets_sent += 1

        return success


    def send_home_axis(self, axis: int) -> bool:
        if not self._is_open: return False
        return self._raw_write(f"{CMD_HOME}:{axis}\n")


    def send_home_all(self) -> bool:
        if not self._is_open: return False
        logger.info("Sending HOMEALL command...")
        self._homing_event.clear()

        if self._raw_write(f"{CMD_HOMEALL}\n"):
            return self._homing_event.wait(timeout=CFG.calibration.homing_timeout_sec)
        return False


    def enable_motors(self) -> bool:
        """Re-enables motor drivers after a HALT."""
        if not self._is_open: return False
        success = self._raw_write(f"{CMD_ENABLE}\n")
        if success:
            self._halted = False
            logger.info("Motors enabled.")
        return success


    def ping(self, timeout_sec: float = 1.0) -> bool:
        if not self._is_open: return False
        self._raw_write(f"{CMD_PING}\n")
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._safe_readline() == RESP_PONG: return True
        return False


    # =========================================================================
    #  Reading State
    # =========================================================================

    def get_latest_sensors(self) -> dict:
        with self._sensor_lock:
            return dict(self._latest_sensors)


    def get_current_angles(self) -> List[float]:
        with self._angles_lock:
            return list(self._current_angles)


    def get_active_session_id(self) -> Optional[int]:
        return self._active_session_id


    def set_active_session_id(self, session_id: Optional[int]):
        self._active_session_id = session_id


    def is_halted(self) -> bool:
        return self._halted


    def is_homed(self) -> List[bool]:
        return list(self._is_homed)


    def get_stats(self) -> dict:
        return {
            "port"            : self._port,
            "is_open"         : self.is_open(),
            "is_halted"       : self._halted,
            "packets_sent"    : self._packets_sent,
            "packets_received": self._packets_received,
        }


    # =========================================================================
    #  Reader Thread Loop
    # =========================================================================

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                line = self._safe_readline()
                if line:
                    self._packets_received += 1
                    self._dispatch_packet(line)
            except serial.SerialException:
                if self._stop_event.is_set(): break
                self._is_open = False
                if self._on_disconnect: self._on_disconnect()
                break
            except Exception:
                time.sleep(0.1)


    def _dispatch_packet(self, line: str):
        if line.startswith(f"{RESP_SENSOR}:"):
            self._handle_sensor_packet(line)
        elif line == RESP_PONG:
            pass
        elif line.startswith(f"{RESP_HOMED}:"):
            self._handle_homed_packet(line)
        elif line == RESP_HOMEDALL:
            self._handle_homedall_packet()
        elif line.startswith(f"{RESP_LIMIT}:"):
            self._handle_limit_packet(line)
        elif line.startswith(f"{RESP_ERROR}:"):
            self._handle_error_packet(line)


    def _handle_sensor_packet(self, line: str):
        try:
            parts = line[2:].split(",")
            if len(parts) == SENSOR_PACKET_FIELDS:
                fsr = float(parts[0])
                emg = float(parts[1])
                ls_states = [int(p) for p in parts[2:]]
                with self._sensor_lock:
                    self._latest_sensors["fsr"] = fsr
                    self._latest_sensors["emg"] = emg
                    self._latest_sensors["limit_switches"] = ls_states
                    self._latest_sensors["timestamp"] = time.monotonic()
                self._check_limit_switches(ls_states)
        except Exception:
            self._parse_errors += 1


    def _handle_homed_packet(self, line: str):
        try:
            axis = int(line.split(":")[1])
            if 1 <= axis <= NUM_AXES:
                with self._angles_lock:
                    self._current_angles[axis - 1] = 0.0
                    self._is_homed[axis - 1] = True
        except Exception: pass


    def _handle_homedall_packet(self):
        with self._angles_lock:
            self._current_angles = [0.0] * NUM_AXES
            self._is_homed = [True] * NUM_AXES
        self._homing_event.set()


    def _handle_limit_packet(self, line: str):
        try:
            parts = line.split(":")
            axis = int(parts[1])
            direction = parts[2]
            if self._on_limit_switch: self._on_limit_switch(axis, direction)
        except Exception: pass


    def _handle_error_packet(self, line: str):
        msg = line[4:]
        if self._on_error: self._on_error(msg)


    def _check_limit_switches(self, current_states: List[int]):
        active = CFG.limit_switches.active_state
        for i, state in enumerate(current_states):
            if state == active and self._prev_limit_states[i] != active:
                axis, direction = self._switch_index_to_axis(i)
                if self._on_limit_switch: self._on_limit_switch(axis, direction)
        self._prev_limit_states = list(current_states)


    def _switch_index_to_axis(self, index: int):
        ls = CFG.limit_switches
        mapping = {
            ls.ax1_min_index: (1, 'min'), ls.ax1_max_index: (1, 'max'),
            ls.ax2_min_index: (2, 'min'), ls.ax2_max_index: (2, 'max'),
            ls.ax3_min_index: (3, 'min'), ls.ax3_max_index: (3, 'max'),
            ls.ax4_min_index: (4, 'min'), ls.ax4_max_index: (4, 'max'),
        }
        return mapping.get(index, (0, 'unknown'))


    def _clamp_to_joint_limits(self, angles: List[float]) -> List[float]:
        j = CFG.joints
        limits = [
            (j.hip_ab_ad.safe_min_deg, j.hip_ab_ad.safe_max_deg),
            (j.hip_flex_ext.safe_min_deg, j.hip_flex_ext.safe_max_deg),
            (j.knee_flex_ext.safe_min_deg, j.knee_flex_ext.safe_max_deg),
            (j.axis4.safe_min_deg, j.axis4.safe_max_deg)
        ]
        return [max(lo, min(hi, a)) for (lo, hi), a in zip(limits, angles)]


    def _raw_write(self, packet: str) -> bool:
        if not self._serial or not self._serial.is_open: return False
        with self._write_lock:
            try:
                self._serial.write(packet.encode('utf-8'))
                self._serial.flush()
                return True
            except Exception: return False


    def _safe_readline(self) -> Optional[str]:
        try:
            raw = self._serial.readline()
            if not raw: return None
            return raw.decode('utf-8', errors='replace').strip()
        except Exception: return None


    def _list_available_ports(self) -> str:
        try:
            ports = serial.tools.list_ports.comports()
            return ", ".join([p.device for p in ports]) if ports else "None"
        except: return "Unknown"