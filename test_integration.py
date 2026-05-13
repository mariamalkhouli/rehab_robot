# =============================================================================
#  test_integration.py
#  /home/rehabrobot/rehab_robot/test_integration.py
#
#  Integration Test — Angles + Emergency Stop + Resume
#
#  WHAT THIS TESTS:
#    1. Pi sends angle commands → Arduino moves motors
#    2. ESP32 button pressed   → HALT sent → motors freeze
#    3. User types RESUME      → ENABLE sent → motors accept commands again
#
#  HOW TO RUN:
#    cd /home/rehabrobot/rehab_robot
#    python test_integration.py
#
#  PACKET FORMAT (matches serial_comm.py and mega.ino exactly):
#    Pi → Arduino: "A:{ax1},{ax2},{ax3},{ax4}\n"
#    Pi → Arduino: "HALT\n"
#    Pi → Arduino: "ENABLE\n"
#    Arduino → Pi: "ACK:MOVING\n", "ACK:HALTED\n", "ACK:ENABLED\n"
#    Arduino → Pi: "S:{fsr},{emg},{ls0},...,{ls7}\n"  (100Hz sensor stream)
#
#  NO PROJECT MODULE DEPENDENCIES — runs completely standalone.
# =============================================================================

import socket
import serial
import threading
import time
import sys

# =============================================================================
#  CONFIGURATION
# =============================================================================

UDP_PORT    = 5005
SERIAL_PORT = "/dev/ttyUSB0"
BAUD        = 115200

# =============================================================================
#  SHARED STATE
# =============================================================================

is_halted   = False          # Mirrors Arduino halt state
state_lock  = threading.Lock()

# =============================================================================
#  SETUP — Serial connection to Arduino
# =============================================================================

print("=" * 55)
print("  Rehab Robot — Integration Test")
print("  Angles + E-Stop + Resume")
print("=" * 55)

print(f"\n[1/2] Connecting to Arduino on {SERIAL_PORT}...")
try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    print("      Waiting 2s for Arduino to boot...")
    time.sleep(2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("      ✅ Serial connected.")
except serial.SerialException as e:
    print(f"      ❌ Failed: {e}")
    sys.exit(1)

# =============================================================================
#  SETUP — UDP socket for ESP32 E-stop
# =============================================================================

print(f"\n[2/2] Opening UDP socket on port {UDP_PORT}...")
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.1)
    sock.bind(("0.0.0.0", UDP_PORT))
    print("      ✅ UDP socket ready.")
except OSError as e:
    print(f"      ❌ Failed: {e}")
    ser.close()
    sys.exit(1)

# =============================================================================
#  HANDSHAKE — verify Arduino is ready
# =============================================================================

print("\n  Performing handshake with Arduino...")
ser.write(b"RPI_READY\n")
handshake_done = False
deadline = time.time() + 5.0

while time.time() < deadline:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line == "MEGA_READY":
            print("  ✅ Arduino handshake confirmed.")
            handshake_done = True
            break
        else:
            print(f"  [Arduino boot] → '{line}'")

if not handshake_done:
    print("  ⚠️  No handshake received — Arduino may still work, continuing...")

# =============================================================================
#  BACKGROUND THREAD — Arduino serial reader
#  Reads all incoming lines from Arduino and prints them.
#  Runs continuously so sensor packets and ACKs don't block the main loop.
# =============================================================================

def arduino_reader():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                # Only print non-sensor packets to avoid flooding terminal
                # Sensor packets (S:...) are printed as a compact status line
                if line.startswith("S:"):
                    parts = line[2:].split(",")
                    if len(parts) == 10:
                        fsr = parts[0]
                        emg = parts[1]
                        ls  = parts[2:]
                        triggered = [i for i, v in enumerate(ls) if v == "1"]
                        ls_str = f"LS triggered: {triggered}" \
                                 if triggered else "LS: all clear"
                        # Overwrite same line — avoids flooding terminal
                        print(
                            f"\r  [Sensor] FSR={fsr:>4}  EMG={emg:>4}  "
                            f"{ls_str}          ",
                            end="", flush=True
                        )
                else:
                    # Print non-sensor messages on their own line
                    print(f"\n  [Arduino] → '{line}'")
            else:
                time.sleep(0.005)
        except Exception:
            break

reader_thread = threading.Thread(target=arduino_reader, daemon=True)
reader_thread.start()

# =============================================================================
#  BACKGROUND THREAD — UDP listener for ESP32 E-stop
# =============================================================================

def udp_listener():
    global is_halted
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()

            if message == "STOP":
                with state_lock:
                    is_halted = True
                print(f"\n\n  🚨 [E-STOP] STOP received from {addr[0]}!")
                print("  ⚡ Sending HALT to Arduino...")
                ser.write(b"HALT\n")
                ser.flush()
                print("  ✅ HALT sent. Motors frozen.")
                print("  → Type 'resume' to re-enable motors.\n")

            elif message == "HB":
                pass  # Heartbeat — ignore silently

        except socket.timeout:
            pass
        except Exception:
            break

udp_thread = threading.Thread(target=udp_listener, daemon=True)
udp_thread.start()

# =============================================================================
#  HELPER — send angle command
# =============================================================================

def send_angles(ax1, ax2, ax3, ax4):
    """
    Sends angle command to Arduino in the standard project format.
    Format: A:{ax1},{ax2},{ax3},{ax4}\n
    """
    packet = f"A:{ax1:.2f},{ax2:.2f},{ax3:.2f},{ax4:.2f}\n"
    ser.write(packet.encode('utf-8'))
    ser.flush()
    print(f"\n  [TX] Sent: '{packet.strip()}'")

# =============================================================================
#  MAIN LOOP — user input
# =============================================================================

print(f"\n{'=' * 55}")
print("  ✅ System ready.")
print()
print("  COMMANDS:")
print("    Enter 4 angles  : e.g.  45 0 90 0")
print("    resume          : re-enable motors after E-stop")
print("    halt            : manually trigger HALT")
print("    status          : show current halt state")
print("    quit            : exit")
print()
print("  Press ESP32 button at any time to test E-stop.")
print(f"{'=' * 55}\n")

try:
    while True:
        try:
            user_input = input(">> ").strip()
        except EOFError:
            break

        if not user_input:
            continue

        # ── quit ──────────────────────────────────────────────────────────
        if user_input.lower() in ['quit', 'exit', 'q']:
            break

        # ── status ────────────────────────────────────────────────────────
        if user_input.lower() == 'status':
            with state_lock:
                halted = is_halted
            state_str = "🔴 HALTED" if halted else "🟢 RUNNING"
            print(f"  System state: {state_str}")
            continue

        # ── manual halt ───────────────────────────────────────────────────
        if user_input.lower() == 'halt':
            with state_lock:
                is_halted = True
            ser.write(b"HALT\n")
            ser.flush()
            print("  🔴 Manual HALT sent to Arduino.")
            continue

        # ── resume after E-stop ───────────────────────────────────────────
        if user_input.lower() == 'resume':
            with state_lock:
                halted = is_halted

            if not halted:
                print("  ⚠️  System is not halted. Nothing to resume.")
                continue

            print("  Sending ENABLE to Arduino...")
            ser.write(b"ENABLE\n")
            ser.flush()

            with state_lock:
                is_halted = False

            print("  ✅ ENABLE sent. Motors can now receive commands.")
            continue

        # ── angle command ─────────────────────────────────────────────────
        parts = user_input.split()
        if len(parts) == 4:
            with state_lock:
                halted = is_halted

            if halted:
                print(
                    "  🔴 System is HALTED. Type 'resume' first "
                    "then send angles."
                )
                continue

            try:
                ax1 = float(parts[0])
                ax2 = float(parts[1])
                ax3 = float(parts[2])
                ax4 = float(parts[3])
            except ValueError:
                print("  ❌ Invalid angles. Enter 4 numbers e.g: 45 0 90 0")
                continue

            # Basic range check before sending
            # These are approximate — exact limits are in config.yaml
            if not all(-180 <= a <= 180 for a in [ax1, ax2, ax3, ax4]):
                print("  ❌ Angle out of range. Keep between -180 and 180.")
                continue

            send_angles(ax1, ax2, ax3, ax4)

        else:
            print(
                "  ❌ Unrecognised command. "
                "Enter 4 angles, 'resume', 'halt', 'status', or 'quit'."
            )

except KeyboardInterrupt:
    print("\n\n  Ctrl+C — shutting down...")

finally:
    print("  Sending HALT to Arduino before closing...")
    try:
        ser.write(b"HALT\n")
        ser.flush()
        time.sleep(0.2)
        ser.close()
    except Exception:
        pass

    try:
        sock.close()
    except Exception:
        pass

    print("  Test complete. Goodbye.")
    print("=" * 55)