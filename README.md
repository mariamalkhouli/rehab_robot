# 4-DOF Smart Portable Robotic Lower-Limb Rehabilitation System

A Raspberry Pi 5 controlled rehabilitation robot targeting Hip and Knee joints.

## Architecture
- **High-level controller:** Raspberry Pi 5 (Flask, IK engine, SQLite, WebSockets)
- **Low-level controller:** Arduino Mega 2560 (AccelStepper, sensor sampling)
- **Safety:** ESP32-C3 wireless E-stop, 8x limit switches, hardware relay

## Therapy Modes
1. Passive CPM
2. Active-Assistive (FSR + EMG fusion)
3. Mimicry / Teach & Replay

## Setup
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 main.py
```

python -c "
from core.config import load_config
load_config()
from web.app import create_app, socketio
app = create_app()
socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)
"