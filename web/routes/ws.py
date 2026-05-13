# =============================================================================
#  web/routes/ws.py
#  /home/pi/rehab_robot/web/routes/ws.py
#
#  WebSocket Real-Time Data Streaming
#
#  EVENTS EMITTED TO CLIENT (server → browser):
#    "sensor_data"    → live FSR, EMG, intent, effort % at ~20Hz
#    "motor_angles"   → live 4-axis positions at ~20Hz
#    "system_status"  → safety state, connection status at ~2Hz
#    "estop_event"    → immediate E-stop notification (from app.notify_estop)
#    "session_tick"   → session elapsed time every second
#
#  EVENTS RECEIVED FROM CLIENT (browser → server):
#    "join_session"   → client joins the live session room
#    "leave_session"  → client leaves the live session room
#    "jog_axis"       → mimicry mode: jog one axis from UI slider
#    "request_status" → client asks for immediate status snapshot
# =============================================================================

import time
import threading
import logging

logger = logging.getLogger(__name__)

# Streaming thread references — one per namespace
_sensor_thread  = None
_status_thread  = None
_streaming      = False


def register_socketio_handlers(socketio):
    """
    Registers all SocketIO event handlers and starts background
    streaming threads. Called once from create_app().
    """
    global _streaming
    _streaming = True

    # ── Client connection events ────────────────────────────────────────────

    @socketio.on("connect")
    def on_connect():
        logger.debug(f"WebSocket client connected.")
        # Send immediate status snapshot on connect
        _emit_status_snapshot(socketio)

    @socketio.on("disconnect")
    def on_disconnect():
        logger.debug("WebSocket client disconnected.")

    @socketio.on("request_status")
    def on_request_status():
        _emit_status_snapshot(socketio)

    @socketio.on("join_session")
    def on_join_session(data):
        """Client joined the live therapy session screen."""
        session_id = data.get("session_id") if data else None
        logger.info(f"Client joined live session room. session_id={session_id}")

    @socketio.on("leave_session")
    def on_leave_session():
        logger.info("Client left live session room.")

    @socketio.on("jog_axis")
    def on_jog_axis(data):
        """
        Mimicry mode: browser UI sends jog command.
        data = { "axis": 1, "steps": 50 }
        """
        from web.app import get_serial_comm
        serial = get_serial_comm()
        if serial and not serial.is_halted():
            axis  = int(data.get("axis",  1))
            steps = int(data.get("steps", 0))
            serial.send_jog(axis, steps)

    # ── Background streaming threads ────────────────────────────────────────

    def sensor_stream_loop():
        """
        Emits sensor_data and motor_angles events at ~20Hz.
        Only runs while clients are connected.
        """
        from core.config import CFG
        from web.app import get_sensor_hub, get_serial_comm

        interval = 1.0 / CFG.websocket.sensor_stream_rate_hz

        while _streaming:
            start = time.monotonic()
            try:
                sensor = get_sensor_hub()
                serial = get_serial_comm()

                # ── Sensor data ────────────────────────────────────────────
                if sensor and sensor.is_running():
                    status = sensor.get_full_status()
                    socketio.emit("sensor_data", {
                        "fsr_raw"           : status.get("fsr_raw",            0),
                        "emg_raw"           : status.get("emg_raw",            0),
                        "emg_rms"           : round(status.get("emg_rms", 0),  1),
                        "fsr_delta"         : status.get("fsr_delta",          0),
                        "intent_direction"  : status.get("intent_direction", "NONE"),
                        "intent_magnitude"  : round(status.get("intent_magnitude", 0), 3),
                        "fsr_gate"          : status.get("fsr_gate",       False),
                        "emg_gate"          : status.get("emg_gate",       False),
                        "intent_confirmed"  : status.get("intent_confirmed", False),
                        "patient_effort_pct": round(status.get("patient_effort_pct", 0), 1),
                        "robot_effort_pct"  : round(100 - status.get("patient_effort_pct", 0), 1),
                        "sensor_stale"      : status.get("sensor_stale",  False),
                        "fsr_rest_raw"      : status.get("fsr_rest_raw",    300),
                        "fsr_push_threshold": status.get("fsr_push_threshold", 150),
                        "fsr_lift_threshold": status.get("fsr_lift_threshold", 100),
                        "emg_threshold"     : status.get("emg_threshold",    80),
                        "ts"                : time.time(),
                    })
                else:
                    # No hardware — emit mock zeros so charts don't break
                    socketio.emit("sensor_data", {
                        "fsr_raw"           : 0,
                        "emg_raw"           : 0,
                        "emg_rms"           : 0,
                        "fsr_delta"         : 0,
                        "intent_direction"  : "NONE",
                        "intent_magnitude"  : 0,
                        "fsr_gate"          : False,
                        "emg_gate"          : False,
                        "intent_confirmed"  : False,
                        "patient_effort_pct": 0,
                        "robot_effort_pct"  : 0,
                        "sensor_stale"      : True,
                        "fsr_rest_raw"      : 300,
                        "fsr_push_threshold": 150,
                        "fsr_lift_threshold": 100,
                        "emg_threshold"     : 80,
                        "ts"                : time.time(),
                    })

                # ── Motor angles ───────────────────────────────────────────
                if serial and serial.is_open():
                    angles = serial.get_current_angles()
                    socketio.emit("motor_angles", {
                        "ax1" : round(angles[0], 2),
                        "ax2" : round(angles[1], 2),
                        "ax3" : round(angles[2], 2),
                        "ax4" : round(angles[3], 2),
                        "ts"  : time.time(),
                    })
                else:
                    socketio.emit("motor_angles", {
                        "ax1": 0, "ax2": 0,
                        "ax3": 0, "ax4": 0,
                        "ts" : time.time(),
                    })

            except Exception as e:
                logger.error(f"Sensor stream error: {e}")

            elapsed   = time.monotonic() - start
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def status_stream_loop():
        """
        Emits system_status at ~2Hz — safety state, connection health.
        """
        from web.app import get_safety_monitor, get_serial_comm

        while _streaming:
            try:
                safety = get_safety_monitor()
                serial = get_serial_comm()

                safety_state = safety.get_status() if safety else {
                    "state": "DISARMED", "is_armed": False,
                    "is_estopped": False, "esp32_ip": None,
                    "heartbeat_age_ms": None, "estop_count": 0,
                }

                serial_state = {
                    "is_open"   : serial.is_open()   if serial else False,
                    "is_halted" : serial.is_halted()  if serial else False,
                    "is_homed"  : serial.is_homed()   if serial else [False]*4,
                }

                socketio.emit("system_status", {
                    "safety" : safety_state,
                    "serial" : serial_state,
                    "ts"     : time.time(),
                })
            except Exception as e:
                logger.error(f"Status stream error: {e}")

            time.sleep(0.5)   # 2Hz

    # Start background threads (daemon — die when main thread exits)
    t1 = threading.Thread(
        target=sensor_stream_loop,
        name="WS-SensorStream",
        daemon=True
    )
    t2 = threading.Thread(
        target=status_stream_loop,
        name="WS-StatusStream",
        daemon=True
    )
    t1.start()
    t2.start()

    logger.info("WebSocket handlers registered. Streaming threads started.")


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def _emit_status_snapshot(socketio):
    """Emits an immediate status snapshot to requesting client."""
    from web.app import get_safety_monitor, get_serial_comm, get_sensor_hub
    try:
        safety = get_safety_monitor()
        serial = get_serial_comm()
        sensor = get_sensor_hub()

        socketio.emit("system_status", {
            "safety": safety.get_status() if safety else {"state": "DISARMED"},
            "serial": {
                "is_open"  : serial.is_open()  if serial else False,
                "is_halted": serial.is_halted() if serial else False,
            },
            "ts": time.time(),
        })
    except Exception as e:
        logger.error(f"Status snapshot error: {e}")