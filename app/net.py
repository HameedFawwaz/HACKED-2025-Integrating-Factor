import socket

import kalman

UDP_IP = ""  # Listen on all available network interfaces
UDP_PORT = 80  # Same as the ESP32 sender port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

process_noise = 0.001
measurement_noise = 0.01
error_covariance = 1.0

"""
while True:
    
    data, addr = sock.recvfrom(1024)  # Buffer size of 1024 bytes
    print(data.decode().split(","))
    """

while True:
    
    data, _ = sock.recvfrom(1024)
    data = data.decode().split(",")
    
    data = list(map(float, data))

    dt = data[12]
    acc_data = [data[6], data[7], data[8]]
    gyro_data = [data[9], data[10], data[11]]
    
    

    kf = kalman.KalmanFilterIMU(dt, process_noise=process_noise, measurement_noise=measurement_noise, error_covariance=error_covariance)

    kf.predict()
    kf.update(acc_data)

    print(kf.get_filtered_acceleration())





