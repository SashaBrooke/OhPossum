import socket
import threading
import time
import uuid
from enum import Enum

class BeaconMode(Enum):
    Discovery = 1
    Connected = 2

# class SystemMode(Enum):
#     Free = 1
#     PositionDemand = 2
#     ObjectTrack = 3

PORT = 9999
DISCOVERY_INTERVAL = 1
BUFFER_SIZE = 1024

# Generate a unique identifier for this beacon instance
BEACON_ID = str(uuid.uuid4())
DISCOVERY_MESSAGE = f"[{BEACON_ID}]:DISCOVERY_PACKET"
CONNECTION_MESSAGE = f"[{BEACON_ID}]:CONNECTED_PACKET"

# Initial mode
beacon_mode = BeaconMode.Discovery

def send_packets():
    global beacon_mode
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        broadcast_address = ("<broadcast>", PORT)
        
        while True:
            try:
                if beacon_mode == BeaconMode.Discovery:
                    sock.sendto(DISCOVERY_MESSAGE.encode(), broadcast_address)
                    print("Discovery packet sent.")
                elif beacon_mode == BeaconMode.Connected:
                    sock.sendto(CONNECTION_MESSAGE.encode(), broadcast_address)
                    print("Connection packet sent.")
            except Exception as e:
                print(f"Error sending packet: {e}")
            
            time.sleep(DISCOVERY_INTERVAL)

def listen_for_packets():
    global beacon_mode
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.bind(("", PORT))
        print(f"Listening for packets on port {PORT}...")

        while True:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
                message = data.decode()

                # Ignore packets from itself
                if message.startswith(f"[{BEACON_ID}]"):
                    continue

                print(f"Received packet from {addr}: {message}")
                
                # Check if packet requests a mode change to Connected
                if "ENTER_CONNECTED_MODE" in message and beacon_mode == BeaconMode.Discovery:
                    beacon_mode = BeaconMode.Connected
                    print("Beacon mode changed to Connected.")
            except Exception as e:
                print(f"Error receiving packet: {e}")

discovery_thread = threading.Thread(target=send_packets, daemon=True)
listening_thread = threading.Thread(target=listen_for_packets, daemon=True)

discovery_thread.start()
listening_thread.start()

discovery_thread.join()
listening_thread.join()