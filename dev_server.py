# =============================================================================
#  dev_server.py
#  /home/rehabrobot/rehab_robot/dev_server.py
#
#  Development server — runs the full web dashboard without any hardware.
#  Use this when Arduino, ESP32, and sensors are not connected.
#
#  HOW TO RUN:
#    cd /home/rehabrobot/rehab_robot
#    source venv/bin/activate
#    python dev_server.py
#
#  Then open:  http://<pi-ip>:5000  or  http://localhost:5000
#
#  WHAT WORKS without hardware:
#    ✓ All pages load correctly
#    ✓ Add / view / edit patients
#    ✓ Session creation and history
#    ✓ Analytics charts (once sessions exist)
#    ✓ Therapist management
#    ✓ Dark / light mode toggle
#    ✓ WebSocket connects (sensor data shows zeros — no hardware)
#    ✓ E-stop button (fires UI overlay, no motor command)
#    ✓ Settings page
#
#  WHAT REQUIRES hardware (Arduino + sensors):
#    ✗ Live motor angle gauges (show 0.00 without Arduino)
#    ✗ Live EMG waveform (flat line without Arduino)
#    ✗ Live FSR bar (no movement without Arduino)
#    ✗ E-stop actually halting motors
#    ✗ Homing sequence
#
#  When hardware is ready, use main.py instead.
# =============================================================================

import sys
import os

# Ensure project root is on path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

print("=" * 55)
print("  RehabOS — Development Server")
print("  No hardware required")
print("=" * 55)

# Step 1 — Load config
try:
    from core.config import load_config
    CFG = load_config()
    print(f"  Config loaded.")
except Exception as e:
    print(f"  [FATAL] Config error: {e}")
    sys.exit(1)

# Step 2 — Init database
try:
    from core.database import init_db, log_system_event
    init_db()
    log_system_event('startup', 'info', 'Dev server started (no hardware).', 'dev_server')
    print(f"  Database ready.")
except Exception as e:
    print(f"  [FATAL] Database error: {e}")
    sys.exit(1)

# Step 3 — Start Flask (no hardware services)
try:
    from web.app import create_app, socketio
    app = create_app(
        serial_comm=None,
        safety_monitor=None,
        sensor_hub=None,
    )
    print(f"  Flask app created.")
    print(f"")
    print(f"  Dashboard → http://<pi-ip>:{CFG.flask.port}")
    print(f"  Dashboard → http://localhost:{CFG.flask.port}")
    print(f"")
    print(f"  Press Ctrl+C to stop.")
    print("=" * 55)

    socketio.run(
        app,
        host=CFG.flask.host,
        port=CFG.flask.port,
        debug=False,
        use_reloader=False,
        log_output=False,
    )
except Exception as e:
    print(f"  [FATAL] Server error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)