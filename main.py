# =============================================================================
#  main.py
#  /home/pi/rehab_robot/main.py
#
#  Entry point for the Rehab Robot system.
#
#  STARTUP SEQUENCE (order is critical — do not change):
#    1. Load and validate config.yaml
#    2. Logging is configured by config loader
#    3. Initialize database (create tables if not exist)
#    4. Start Safety Monitor thread  ← MUST be before any motor/serial activity
#    5. Open serial connection to Arduino Mega + handshake
#    6. Start Sensor Hub polling thread
#    7. Start Flask-SocketIO web server  ← blocks main thread, always last
#
#  SHUTDOWN SEQUENCE (triggered by Ctrl+C or SIGTERM):
#    1. Signal all threads to stop
#    2. Send HALT to Arduino Mega
#    3. Close serial port
#    4. Stop Safety Monitor
#    5. Log shutdown event
#    6. Exit cleanly
# =============================================================================


import sys
import os
import time
import signal
import logging
import threading

# ---------------------------------------------------------------------------
# Ensure the project root is on sys.path regardless of where Python is invoked
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# ---------------------------------------------------------------------------
# Step 1 — Load config FIRST. Nothing else runs before this.
# ---------------------------------------------------------------------------
try:
    from core.config import load_config
    CFG = load_config()
except FileNotFoundError as e:
    print(f"\n[FATAL] {e}")
    print("Create config.yaml at the project root before running.\n")
    sys.exit(1)
except ValueError as e:
    print(f"\n[FATAL] Configuration error:\n{e}\n")
    sys.exit(1)
except Exception as e:
    print(f"\n[FATAL] Unexpected error loading config: {e}\n")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Logging is now active (configured inside load_config).
# All subsequent code uses the logger.
# ---------------------------------------------------------------------------
logger = logging.getLogger(__name__)
logger.info("=" * 65)
logger.info(f"  {CFG.system.device_name} — SYSTEM STARTING")
logger.info("=" * 65)

# ---------------------------------------------------------------------------
# Step 2 — Initialize database
# ---------------------------------------------------------------------------
try:
    from core.database import init_db, log_system_event
    init_db()
    logger.info("Database initialized successfully.")
    log_system_event(
        event_type='startup',
        severity='info',
        message='System startup initiated.',
        source_module='main'
    )
except Exception as e:
    logger.critical(f"Database initialization failed: {e}", exc_info=True)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Step 3 — Import core modules
# These are imported here (not at top of file) so that logging is already
# active when they run their module-level code.
# ---------------------------------------------------------------------------
try:
    from core.safety_monitor import SafetyMonitor
    from core.serial_comm    import SerialComm
    from core.sensor_hub     import SensorHub
    logger.info("Core modules imported successfully.")
except ImportError as e:
    logger.critical(f"Failed to import core module: {e}", exc_info=True)
    log_system_event('startup', 'critical', f"Core import failed: {e}", 'main')
    sys.exit(1)

# ---------------------------------------------------------------------------
# Import Flask app factory
# ---------------------------------------------------------------------------
try:
    from web.app import create_app, socketio
    logger.info("Web application module imported successfully.")
except ImportError as e:
    logger.critical(f"Failed to import web app: {e}", exc_info=True)
    log_system_event('startup', 'critical', f"Web app import failed: {e}", 'main')
    sys.exit(1)

# =============================================================================
#  Global references to running services.
#  Held at module level so the shutdown handler can reach them.
# =============================================================================
_safety_monitor : SafetyMonitor = None   # type: ignore
_serial_comm    : SerialComm    = None   # type: ignore
_sensor_hub     : SensorHub     = None   # type: ignore
_session_manager = None   # type: ignore  # Forward reference for type hinting
_shutdown_event : threading.Event = threading.Event()


# =============================================================================
#  SHUTDOWN HANDLER
#  Called on Ctrl+C (SIGINT) or kill signal (SIGTERM).
#  Must stop everything cleanly and in the right order.
# =============================================================================

def _shutdown(signum=None, frame=None):
    """
    Graceful shutdown handler.
    Stops all threads, sends HALT to Arduino, closes serial, logs shutdown.
    Safe to call multiple times — uses _shutdown_event to prevent double execution.
    """
    if _shutdown_event.is_set():
        return   # already shutting down
    _shutdown_event.set()

    logger.info("=" * 65)
    logger.info("  SHUTDOWN SIGNAL RECEIVED — shutting down gracefully...")
    logger.info("=" * 65)

    # 1. Stop sensor hub first — it reads from serial, must stop before serial closes
    if _sensor_hub is not None:
        try:
            logger.info("Stopping Sensor Hub...")
            _sensor_hub.stop()
            logger.info("Sensor Hub stopped.")
        except Exception as e:
            logger.error(f"Error stopping Sensor Hub: {e}")

    # 2. Send HALT to Arduino and close serial port
    if _serial_comm is not None:
        try:
            logger.info("Sending HALT to Arduino Mega...")
            _serial_comm.send_halt()
            time.sleep(0.2)   # Give Arduino time to process HALT
            logger.info("Closing serial port...")
            _serial_comm.close()
            logger.info("Serial port closed.")
        except Exception as e:
            logger.error(f"Error during serial shutdown: {e}")

    # 3. Stop safety monitor last — it must remain active until all motors are stopped
    if _safety_monitor is not None:
        try:
            logger.info("Stopping Safety Monitor...")
            _safety_monitor.stop()
            logger.info("Safety Monitor stopped.")
        except Exception as e:
            logger.error(f"Error stopping Safety Monitor: {e}")

    # 4. Log clean shutdown to database
    try:
        log_system_event(
            event_type='shutdown',
            severity='info',
            message='System shutdown completed cleanly.',
            source_module='main'
        )
    except Exception:
        pass   # Database may be unavailable at this point — that's okay

    logger.info("System shutdown complete. Goodbye.")
    # sys.exit() is NOT called here — Flask's own shutdown handles process exit


# Register shutdown handler for both Ctrl+C and kill signals
signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)


# =============================================================================
#  STARTUP FUNCTIONS
#  Each step is isolated in its own function with clear success/failure logging.
# =============================================================================

def _start_safety_monitor() -> SafetyMonitor:
    """
    Step 4 — Start the Safety Monitor.
    This MUST succeed before any serial or motor activity begins.
    Returns the running SafetyMonitor instance.
    """
    logger.info("Starting Safety Monitor (UDP heartbeat listener)...")
    logger.info(f"  Listening on port : {CFG.safety.heartbeat_port}")
    logger.info(f"  Heartbeat timeout : {CFG.safety.heartbeat_timeout_ms} ms")

    try:
        monitor = SafetyMonitor(
            port=CFG.safety.heartbeat_port,
            timeout_ms=CFG.safety.heartbeat_timeout_ms,
            on_estop_callback=_on_estop_triggered
        )
        monitor.start()

        # Give the monitor thread a moment to bind the socket and confirm running
        time.sleep(0.5)

        if not monitor.is_running():
            raise RuntimeError("Safety Monitor thread started but is not in running state.")

        logger.info("Safety Monitor is ACTIVE. E-stop watchdog is live.")
        log_system_event(
            event_type='startup',
            severity='info',
            message=f"Safety Monitor started on UDP port {CFG.safety.heartbeat_port}.",
            source_module='main'
        )
        return monitor

    except Exception as e:
        logger.critical(f"FAILED to start Safety Monitor: {e}", exc_info=True)
        log_system_event(
            'startup', 'critical',
            f"Safety Monitor failed to start: {e}", 'main'
        )
        raise   # Propagate — system cannot run without safety monitor


def _start_serial_comm() -> SerialComm:
    """
    Step 5 — Open serial connection to Arduino Mega and perform handshake.
    Returns the connected SerialComm instance.
    """
    logger.info("Opening serial connection to Arduino Mega...")
    logger.info(f"  Port      : {CFG.serial.port}")
    logger.info(f"  Baud rate : {CFG.serial.baud_rate}")

    try:
        comm = SerialComm(
            port=CFG.serial.port,
            baud_rate=CFG.serial.baud_rate,
            timeout_sec=CFG.serial.timeout_sec
        )
        comm.open()

        # Perform handshake — Pi sends token, waits for Mega's acknowledgement
        logger.info(f"  Performing handshake (sending: '{CFG.serial.handshake_token}')...")
        success = comm.handshake(
            send_token=CFG.serial.handshake_token,
            expected_ack=CFG.serial.handshake_ack,
            retries=CFG.serial.handshake_retries,
            delay_sec=CFG.serial.handshake_delay_sec
        )

        if not success:
            raise RuntimeError(
                f"Arduino handshake failed after {CFG.serial.handshake_retries} attempts. "
                f"Check USB cable, Arduino power, and that firmware is flashed."
            )

        logger.info("Serial connection established. Arduino Mega is responsive.")
        log_system_event(
            event_type='startup',
            severity='info',
            message=f"Serial connected: {CFG.serial.port} @ {CFG.serial.baud_rate} baud.",
            source_module='main'
        )
        return comm

    except Exception as e:
        logger.critical(f"FAILED to open serial connection: {e}", exc_info=True)
        log_system_event(
            'startup', 'critical',
            f"Serial connection failed: {e}", 'main'
        )
        raise


def _start_sensor_hub(serial_comm: SerialComm) -> SensorHub:
    """
    Step 6 — Start the Sensor Hub polling thread.
    Reads FSR + EMG data from the serial stream and maintains latest values.
    Returns the running SensorHub instance.
    """
    logger.info("Starting Sensor Hub...")
    logger.info(f"  Sample rate : {CFG.sensors.sample_rate_hz} Hz")
    logger.info(f"  RMS window  : {CFG.sensors.emg_rms_window_samples} samples "
                f"({CFG.sensors.rms_window_sec:.2f} sec)")

    try:
        hub = SensorHub(serial_comm=serial_comm)
        hub.start()

        # Brief wait to confirm the thread is polling
        time.sleep(0.3)

        if not hub.is_running():
            raise RuntimeError("Sensor Hub thread started but is not in running state.")

        logger.info("Sensor Hub is ACTIVE.")
        log_system_event(
            event_type='startup',
            severity='info',
            message="Sensor Hub started.",
            source_module='main'
        )
        return hub

    except Exception as e:
        logger.critical(f"FAILED to start Sensor Hub: {e}", exc_info=True)
        log_system_event(
            'startup', 'critical',
            f"Sensor Hub failed to start: {e}", 'main'
        )
        raise


def _start_flask(serial_comm: SerialComm,
                 safety_monitor: SafetyMonitor,
                 sensor_hub: SensorHub):
    """
    Step 7 — Create and start the Flask-SocketIO web server.
    This call BLOCKS — it must be the last thing called in main().
    The Flask app receives references to all running services so routes
    and WebSocket handlers can communicate with hardware.
    """
    try:
        from therapy.session_manager import SessionManager
        session_manager = SessionManager(serial_comm, sensor_hub)
        
        logger.info("Starting Flask-SocketIO web server...")
        logger.info(f"  Host    : {CFG.flask.host}")
        logger.info(f"  Port    : {CFG.flask.port}")
        logger.info(f"  Debug   : {CFG.flask.debug}")
        logger.info(
            f"  Dashboard URL: http://<pi-ip>:{CFG.flask.port}  "
            f"or  http://rehabrobot.local:{CFG.flask.port}"
        )

        app = create_app(
            serial_comm=serial_comm,
            safety_monitor=safety_monitor,
            sensor_hub=sensor_hub,
            session_manager=session_manager # <--- CHANGE 'manager_obj' TO 'session_manager'



        )

        log_system_event(
            event_type='startup',
            severity='info',
            message=(
                f"Flask server starting on {CFG.flask.host}:{CFG.flask.port}. "
                f"System fully operational."
            ),
            source_module='main'
        )

        logger.info("=" * 65)
        logger.info("  SYSTEM FULLY OPERATIONAL")
        logger.info(f"  Open dashboard at: http://<pi-ip>:{CFG.flask.port}")
        logger.info("  Press Ctrl+C to shut down.")
        logger.info("=" * 65)

        # This call blocks until shutdown
        socketio.run(
            app,
            host=CFG.flask.host,
            port=CFG.flask.port,
            debug=False,
            use_reloader=False,      # CRITICAL: reloader spawns child process
                                     # which would re-run startup sequence
                                     # and double-open serial port → crash
            log_output=True         # Silence SocketIO's own request logs
                                     # (our logger handles all output)
        )

    except Exception as e:
        logger.critical(f"Flask server crashed: {e}", exc_info=True)
        print(f"ACTUAL ERROR: {e}") # <--- Add this line to see the bug
        import traceback
        print("\n" + "!"*60)
        print("FLASK CRASHED! HERE IS THE SPECIFIC ERROR:")
        traceback.print_exc() 
        print("!"*60 + "\n")
        log_system_event(
            'startup', 'critical',
            f"Flask server failed: {e}", 'main'
        )
        raise


# =============================================================================
#  E-STOP CALLBACK
#  Called by SafetyMonitor when a heartbeat is lost or button is pressed.
#  This runs inside the SafetyMonitor thread — keep it fast and thread-safe.
# =============================================================================

def _on_estop_triggered(source: str, reason: str):
    """
    Called by SafetyMonitor the moment an E-stop condition is detected.

    Args:
        source : 'heartbeat_loss' | 'wireless_button' | 'limit_switch' | 'software'
        reason : Human-readable description of why E-stop was triggered
    """
    logger.critical(f"!!! E-STOP TRIGGERED !!! Source: {source} | Reason: {reason}")

    # 1. Send HALT to Arduino immediately
    if _serial_comm is not None:
        try:
            _serial_comm.send_halt()
        except Exception as e:
            logger.error(f"Failed to send HALT during E-stop: {e}")

    # 2. Log to database
    try:
        from core.database import log_estop

        # Get current motor positions if available
        angles = None
        if _serial_comm is not None:
            try:
                angles = _serial_comm.get_current_angles()
            except Exception:
                pass

        ax1 = angles[0] if angles else None
        ax2 = angles[1] if angles else None
        ax3 = angles[2] if angles else None
        ax4 = angles[3] if angles else None

        # Get current session ID from serial_comm if available
        session_id = None
        if _serial_comm is not None:
            try:
                session_id = _serial_comm.get_active_session_id()
            except Exception:
                pass

        log_estop(
            session_id=session_id,
            trigger_source=source,
            trigger_reason=reason,
            ax1=ax1, ax2=ax2, ax3=ax3, ax4=ax4
        )
    except Exception as e:
        logger.error(f"Failed to log E-stop to database: {e}")

    # 3. Notify connected dashboard clients via WebSocket (non-blocking)
    try:
        from web.app import notify_estop
        notify_estop(source=source, reason=reason)
    except Exception as e:
        logger.error(f"Failed to notify dashboard of E-stop: {e}")


# =============================================================================
#  MAIN
# =============================================================================

def main():
    global _safety_monitor, _serial_comm, _sensor_hub

    logger.info("Beginning startup sequence...")

    # ── Step 4: Safety Monitor ───────────────────────────────────────────────
    try:
        _safety_monitor = _start_safety_monitor()
    except Exception:
        logger.critical("Startup aborted: Safety Monitor could not start.")
        sys.exit(1)

    # ── Step 5: Serial Communication ─────────────────────────────────────────
    try:
        _serial_comm = _start_serial_comm()
    except Exception:
        logger.critical("Startup aborted: Serial connection could not be established.")
        _shutdown()
        sys.exit(1)

    # ── Step 6: Sensor Hub ───────────────────────────────────────────────────
    try:
        _sensor_hub = _start_sensor_hub(_serial_comm)
    except Exception:
        logger.critical("Startup aborted: Sensor Hub could not start.")
        _shutdown()
        sys.exit(1)

    # ── Step 7: Flask Web Server (BLOCKS) ────────────────────────────────────
    try:
        _start_flask(
            serial_comm=_serial_comm,
            safety_monitor=_safety_monitor,
            sensor_hub=_sensor_hub
        )
    except Exception:
        logger.critical("Flask server failed. Initiating shutdown.")
        _shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
