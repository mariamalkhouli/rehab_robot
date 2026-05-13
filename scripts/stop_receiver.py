import socket

# UDP Setup - Must match ESP32 code
UDP_IP = "0.0.0.0" # Listen on all available interfaces
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"✅ Pi Receiver Active. Listening on Port {UDP_PORT}...")
print("Waiting for Emergency Stop signal...")

try:
    while True:
        data, addr = sock.recvfrom(1024) # Buffer size 1024 bytes
        message = data.decode('utf-8')
        
        if message == "STOP":
            print(f"🚨 [EMERGENCY] STOP received from {addr[0]}!")
            # In the final project, this is where we tell the Mega to cut power.
        else:
            print(f"Received unknown packet: {message}")

except KeyboardInterrupt:
    print("\nShutting down receiver...")
    sock.close()