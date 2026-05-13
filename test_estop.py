import socket
import serial
import threading
import time
import sys

# Config
UDP_PORT = 5005
SERIAL_PORT = "/dev/ttyUSB0" # Change to /dev/ttyACM0 if needed
BAUD = 115200

# Safety State
stop_active = False

# Setup Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    time.sleep(2)
    print("✅ Connected to Arduino Mega")
except Exception as e:
    print(f"❌ Serial Error: {e}")
    sys.exit()

# Thread: Listen for ESP32 UDP "STOP"
def udp_listener():
    global stop_active
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024)
        if data.decode('utf-8').strip() == "STOP":
            print(f"\n🚨 [UDP] STOP RECEIVED FROM {addr[0]}!")
            stop_active = True
            ser.write(b"HALT\n")
            print("⚠️ Sent HALT to Arduino. Power Disconnected.")

threading.Thread(target=udp_listener, daemon=True).start()

# Main Loop: User Input
print("\n--- Rehab Robot Master Control ---")
print("Enter 4 angles (e.g. 45.5 0 90 0) or Ctrl+C to quit")

try:
    while not stop_active:
        user_input = input(">> Target Angles: ")
        
        if stop_active: break # Exit if button was pressed during input
        
        angles = user_input.split()
        if len(angles) == 4:
            # Send in format <A1,A2,A3,A4>
            packet = f"<{angles[0]},{angles[1]},{angles[2]},{angles[3]}>\n"
            ser.write(packet.encode())
            
            # Read feedback from Arduino
            time.sleep(0.1)
            if ser.in_waiting > 0:
                print(f"   [Mega]: {ser.readline().decode().strip()}")
        else:
            print("   Error: Enter exactly 4 numbers.")

except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    ser.close()