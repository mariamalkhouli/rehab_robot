import sys
import os

# 1. Add project root to path so Python can find 'core' and 'web'
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from core.config import load_config
from core.database import init_db
from web.app import create_app, socketio

def main():
    print("--- REHAB ROBOT DEV SERVER ---")
    
    # 2. Initialize Config (This is what was missing!)
    # This sets the global CFG object used by all other modules.
    print("[1/3] Loading configuration...")
    try:
        load_config()
    except Exception as e:
        print(f"FAILED to load config: {e}")
        return

    # 3. Initialize Database
    # Creates rehab.db if it doesn't exist so the API doesn't crash.
    print("[2/3] Initializing database...")
    init_db()

    # 4. Create App with NO hardware
    print("[3/3] Creating Flask instance (Mock Hardware Mode)...")
    app = create_app(
        serial_comm=None,
        safety_monitor=None,
        sensor_hub=None
    )

    print("\n✅ Web App ready at: http://localhost:5000")
    print("Press Ctrl+C to stop.\n")

    # 5. Run with debug=True to see errors if it crashes later
    socketio.run(
        app, 
        host="0.0.0.0", 
        port=5000, 
        debug=True,       # Change to True for development!
        use_reloader=False
    )

if __name__ == "__main__":
    main()