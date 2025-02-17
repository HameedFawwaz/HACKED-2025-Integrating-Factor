import pygame
import math
import random
import socket

import serial
import nav

import time
import kalman

# Initialize pygame
pygame.init()


#arduino = serial.Serial(port="COM5", baudrate=9600, timeout=0.1)


# Constants
WIDTH, HEIGHT = 600, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
RADIUS = 250  # Radius of the circular display

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

UDP_IP = ""  # Listen on all available network interfaces
UDP_PORT = 80  # Same as the ESP32 sender port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")


#Create Sample Data

dat = nav.Data()
 
# Create screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Circular GPS Display")

# Variables
running = True
angle_offset = 0  # Rotation angle
scale_factor = 1.0  # Zoom factor
#points = []  # List of points to plot

# Function to rotate a point around the center
def rotate_point(x, y, angle, scale):
    angle_rad = math.radians(angle)
    x = CENTER[0] - float(x)
    y = CENTER[1] - float(y)

    x *= scale
    y *= scale

    x_new = (x * math.cos(angle_rad) - y * math.sin(angle_rad))
    y_new = (x * math.sin(angle_rad) + y * math.cos(angle_rad))
    return int(x_new + CENTER[0]), int(y_new + CENTER[1])
"""
def read_imu(data):
    acc = (data[0], data[1], data[2])
    vel = (data[3], data[4], data[5])
    pos = (data[6], data[7], data[8])

    w = (data[9], data[10], data[11])
    theta = (data[12], data[13], data[14])



    dat.update_data(acc, vel, pos, w, theta)
"""
i = 0

#Constants for Kalman Filter

process_noise = 0.001
measurement_noise = 0.01
error_covariance = 1.0

# Main loop
while running:

    i += 1

    screen.fill(WHITE)
    
    # Draw circular boundary
    pygame.draw.circle(screen, BLACK, CENTER, RADIUS, 3)

    data, _ = sock.recvfrom(1024)
    data = data.decode("utf-8").split(",")
    data = list(map(float, data))
    print(data)


    dt = data[12]
    acc_data = [data[6], data[7], data[8]]
    gyro_data = [data[9], data[10], data[11]]
    pos_data = (data[0], data[1], data[2])
    vel_data = [data[3], data[4], data[5]]
    

    dat.data["pos"].append(pos_data)
    points = dat.data["pos"]

    # Extract points and convert NumPy arrays to tuples
    points = [list(p) for p in dat.data["pos"]]


    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                angle_offset += 5  # Rotate left
            elif event.key == pygame.K_RIGHT:
                angle_offset -= 5  # Rotate right
            elif event.key == pygame.K_SPACE:
                # Add a random point within the circle for testing
                new_x = CENTER[0] + (math.cos(math.radians(angle_offset)) * (RADIUS // 2))
                new_y = CENTER[1] + (math.sin(math.radians(angle_offset)) * (RADIUS // 2))
                points.append((new_x, new_y, 0))
            elif event.key == pygame.K_UP:
                scale_factor *= 1.1  # Zoom in (increase scale)
            elif event.key == pygame.K_DOWN:
                scale_factor /= 1.1  # Zoom out (decrease scale)
    
    # Draw points after rotating
    prev_point = None
    for p in points:
        px, py, _ = p  # Unpack x and y (ignore z)
        rotated_x, rotated_y = rotate_point(px, py, angle_offset, scale_factor)

        if math.hypot(rotated_x - CENTER[0], rotated_y - CENTER[1]) <= RADIUS:
            pygame.draw.circle(screen, RED, (rotated_x, rotated_y), 5)

            if prev_point is not None:
                pygame.draw.line(screen, BLACK, prev_point, (rotated_x, rotated_y), 2)

            prev_point = (rotated_x, rotated_y)  # Store current point as previous
    pygame.display.flip()

pygame.quit()
