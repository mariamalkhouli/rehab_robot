# =============================================================================
#  web/app.py
#  /home/pi/rehab_robot/web/app.py
#
#  Flask Application Factory
#
#  WHAT THIS DOES:
#    - Creates and configures the Flask app
#    - Initializes Flask-SocketIO
#    - Registers all route blueprints
#    - Holds references to hardware services (serial_comm, safety_monitor,
#      sensor_hub) so every route and WebSocket handler can reach them
#    - Exposes notify_estop() so main.py can push E-stop events to
#      connected dashboard clients immediately
#
#  USAGE (from main.py):
#    from web.app import create_app, socketio
#    app = create_app(serial_comm, safety_monitor, sensor_hub)
#    socketio.run(app, ...)
# =============================================================================

import logging
from flask import Flask
from flask_socketio import SocketIO

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
#  SocketIO instance — created here, shared across all route modules
#  eventlet async mode gives best performance on Pi
# ---------------------------------------------------------------------------
socketio = SocketIO(
    cors_allowed_origins="*",
    async_mode="threading",   # threading mode — safe, no eventlet required
    logger=False,
    engineio_logger=False
)

# ---------------------------------------------------------------------------
#  Service registry — populated by create_app(), read by route modules
# ---------------------------------------------------------------------------
_services: dict = {
    "serial_comm"    : None,
    "safety_monitor" : None,
    "sensor_hub"     : None,
    "session_manager": None,
}

def get_session_manager():
    return _services["session_manager"]


def get_serial_comm():
    return _services["serial_comm"]

def get_safety_monitor():
    return _services["safety_monitor"]

def get_sensor_hub():
    return _services["sensor_hub"]


# ---------------------------------------------------------------------------
#  Application factory
# ---------------------------------------------------------------------------

def create_app(
    serial_comm=None,
    safety_monitor=None,
    sensor_hub=None,
    session_manager=None,
) -> Flask:
    """
    Creates and fully configures the Flask application.

    Args:
        serial_comm    : Running SerialComm instance (can be None during dev)
        safety_monitor : Running SafetyMonitor instance (can be None during dev)
        sensor_hub     : Running SensorHub instance (can be None during dev)

    Returns:
        Configured Flask app (not yet running — call socketio.run() after)
    """
    from core.config import CFG

    # Store service references
    _services["serial_comm"]    = serial_comm
    _services["safety_monitor"] = safety_monitor
    _services["sensor_hub"]     = sensor_hub
    _services["session_manager"] = session_manager

    # Create Flask app
    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static"
    )

    app.config["SECRET_KEY"]       = CFG.flask.secret_key
    app.config["MAX_CONTENT_LENGTH"] = CFG.flask.max_content_length_mb * 1024 * 1024

    # Initialize SocketIO with the app
    socketio.init_app(app)

    # Register blueprints
    from web.routes.api   import api_bp
    from web.routes.pages import pages_bp
    from web.routes.ws    import register_socketio_handlers

    app.register_blueprint(api_bp)
    app.register_blueprint(pages_bp)
    register_socketio_handlers(socketio)

    logger.info("Flask application created and configured.")
    return app


# ---------------------------------------------------------------------------
#  notify_estop — called from main.py E-stop callback
#  Pushes E-stop event to all connected dashboard clients immediately
# ---------------------------------------------------------------------------

def notify_estop(source: str, reason: str):
    """
    Emits an E-stop event to all connected WebSocket clients.
    Called from main.py _on_estop_triggered() the moment E-stop fires.
    Every open dashboard tab will immediately show the E-stop alert.

    Guard: if Flask-SocketIO server has not started yet (e.g. E-stop fires
    during startup before socketio.run() is called), the emit is silently
    skipped. The dashboard will still show E-stop state via the system_status
    broadcast once the server starts.
    """
    try:
        # Check server is running before emitting
        # socketio.server is None until socketio.run() has been called
        if socketio.server is None:
            logger.warning(
                "notify_estop: SocketIO server not yet started — "
                "skipping emit. Dashboard will reflect state on next poll."
            )
            return
        socketio.emit("estop_event", {
            "source" : source,
            "reason" : reason,
        })
        logger.info("E-stop event broadcast to dashboard clients.")
    except Exception as e:
        logger.error(f"Failed to emit E-stop event: {e}")