import socket
import threading
import time
import uuid
from enum import Enum

class ProcessorMode(Enum):
    Disconnected = 1
    FoundPotential = 2
    Connected = 3

# class SystemMode(Enum):
#     Free = 1
#     PositionDemand = 2
#     ObjectTrack = 3

PORT = 9999
SEND_INTERVAL = 1 # TBD
BUFFER_SIZE = 1024

# Generate a unique identifier for this processor instance
PROCESSOR_ID = str(uuid.uuid4())
CONNECTION_COMMAND = f"[{PROCESSOR_ID}]:ENTER_CONNECTED_MODE"
CONNECTED_MESSAGE = f"[{PROCESSOR_ID}]:CONNECTED"

# Initial mode
processor_mode = ProcessorMode.Disconnected
beaconIP = "" # set during discovery process

def send_packets():
    global processor_mode
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        broadcast_address = (beaconIP, PORT)
        
        while True:
            try:
                if processor_mode == ProcessorMode.FoundPotential:
                    sock.sendto(CONNECTION_COMMAND.encode(), broadcast_address)
                    print("Connection command sent.")
                elif processor_mode == ProcessorMode.Connected:
                    sock.sendto(CONNECTED_MESSAGE.encode(), broadcast_address)
                    print("Connected message sent.")
            except Exception as e:
                print(f"Error sending packet: {e}")
            
            time.sleep(SEND_INTERVAL)

def listen_for_packets():
    global processor_mode, beaconIP
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        while True:
            if processor_mode == ProcessorMode.Disconnected:
                sock.bind(("", PORT))
                print(f"Listening for packets from any IP on port {PORT}...")
            elif processor_mode == ProcessorMode.FoundPotential or processor_mode == ProcessorMode.Connected:
                sock.bind((beaconIP, PORT))
                print(f"Listening for packets from {beaconIP} on port {PORT}...")

            try:
                # Receive data from any sender
                data, addr = sock.recvfrom(BUFFER_SIZE)
                message = data.decode()

                # Ignore packets from itself
                if message.startswith(f"[{PROCESSOR_ID}]"):
                    continue

                print(f"Received packet from {addr}: {message}")
                
                if processor_mode == ProcessorMode.Disconnected:
                    processor_mode = ProcessorMode.FoundPotential
                    beaconIP = addr[0]
                    print("Found potential beacon.")
                elif "CONNECTED_PACKET" in message and processor_mode == ProcessorMode.FoundPotential:
                    processor_mode = ProcessorMode.Connected
                    print(f"Connected to beacon at {beaconIP}.")
                elif processor_mode == ProcessorMode.Connected:
                    print("Connected.")
            except Exception as e:
                print(f"Error receiving packet: {e}")

discovery_thread = threading.Thread(target=send_packets, daemon=True)
listening_thread = threading.Thread(target=listen_for_packets, daemon=True)

discovery_thread.start()
listening_thread.start()

discovery_thread.join()
listening_thread.join()