import socket

import kalman

import time

UDP_IP = ""  # Listen on all available network interfaces
UDP_PORT = 80  # Same as the ESP32 sender port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

while True:
    
    data, _ = sock.recvfrom(1024)
    data = data.decode()

    if not data.startswith("Help!"):

        data = data.split(",")
        
        data = list(map(float, data))
    
    else:
        print("Distress Signal Detected!")
