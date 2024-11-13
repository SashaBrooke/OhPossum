import socket
import threading
import time
import uuid

PORT = 9999
DISCOVERY_INTERVAL = 1
BUFFER_SIZE = 1024

# Generate a unique identifier for this beacon instance
BEACON_ID = str(uuid.uuid4())
DISCOVERY_MESSAGE = f"[{BEACON_ID}]:DISCOVERY_PACKET"

def send_discovery_packets():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        broadcast_address = ("255.255.255.255", PORT)
        
        while True:
            try:
                sock.sendto(DISCOVERY_MESSAGE.encode(), broadcast_address)
                print("Discovery packet sent.")
            except Exception as e:
                print(f"Error sending discovery packet: {e}")
            
            time.sleep(DISCOVERY_INTERVAL)

def listen_for_packets():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.bind(("", PORT))
        
        print(f"Listening for packets on port {PORT}...")

        while True:
            try:
                # Receive data from any sender
                data, addr = sock.recvfrom(BUFFER_SIZE)
                message = data.decode()

                # Check if the received message starts with the BEACON_ID and ignore if it does
                if not message.startswith(f"[{BEACON_ID}]"):
                    print(f"Received packet from {addr}: {message}")
            except Exception as e:
                print(f"Error receiving packet: {e}")

discovery_thread = threading.Thread(target=send_discovery_packets, daemon=True)
listening_thread = threading.Thread(target=listen_for_packets, daemon=True)

discovery_thread.start()
listening_thread.start()

discovery_thread.join()
listening_thread.join()