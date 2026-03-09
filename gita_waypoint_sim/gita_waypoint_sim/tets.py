# test_receiver.py
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 8888))
print("Listening on port 8888...")
print("Waiting for ANY data...")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"✓ Received from {addr}: {data.decode('utf-8')[:100]}")