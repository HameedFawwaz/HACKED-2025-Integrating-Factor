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
PAN_SPEED = 100

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
pygame.display.set_caption("F.E.I.N")

# Variables
running = True
angle_offset = 0  # Rotation angle
scale_factor = 1.0  # Zoom factor
#points = []  # List of points to plot
pan_x, pan_y = 0, 0
panning = False
mouse_start = (0, 0)

# Font for title and labels
font = pygame.font.Font(None, 36)
import pygame
import math

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 600, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
RADIUS = 250  # Radius of the circular display

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# Create screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("F.E.I.N")

# Font for labels
font = pygame.font.Font(None, 36)

# Function to draw dotted lines stopping at the circle
def draw_dotted_line(color, start_pos, end_pos, dash_length=5):
    x1, y1 = start_pos
    x2, y2 = end_pos
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)
    dashes = int(dist // (2 * dash_length))

    for i in range(dashes):
        start_x = x1 + (dx / dashes) * i * 2
        start_y = y1 + (dy / dashes) * i * 2
        end_x = start_x + (dx / dashes)
        end_y = start_y + (dy / dashes)
        pygame.draw.line(screen, color, (start_x, start_y), (end_x, end_y), 2)

# Function to draw axes with arrowheads
def draw_axes():
    # Find precise points where axes intersect the circle
    x_edge = int(math.sqrt(RADIUS ** 2))  # Intersection of X-axis (y=0)
    y_edge = int(math.sqrt(RADIUS ** 2))  # Intersection of Y-axis (x=0)

    # X-axis (dotted line from left to right edge)
    draw_dotted_line(BLACK, (CENTER[0] - x_edge, CENTER[1]), (CENTER[0] + x_edge - 250, CENTER[1]))

    # Y-axis (dotted line from bottom to top edge)
    draw_dotted_line(BLACK, (CENTER[0], CENTER[1] + y_edge), (CENTER[0], CENTER[1] - y_edge + 250))

    # Draw arrowheads
    pygame.draw.polygon(screen, BLACK, [(CENTER[0] + x_edge, CENTER[1]), (CENTER[0] + x_edge - 10, CENTER[1] - 5), (CENTER[0] + x_edge - 10, CENTER[1] + 5)])  # X arrow
    pygame.draw.polygon(screen, BLACK, [(CENTER[0], CENTER[1] - y_edge), (CENTER[0] - 5, CENTER[1] - y_edge + 10), (CENTER[0] + 5, CENTER[1] - y_edge + 10)])  # Y arrow

    # Labels
    x_label = font.render("X", True, BLACK)
    y_label = font.render("Y", True, BLACK)
    screen.blit(x_label, (CENTER[0] + x_edge - 15, CENTER[1] + 5))
    screen.blit(y_label, (CENTER[0] + 5, CENTER[1] - y_edge + 5))


# Function to rotate a point around the center
def rotate_point(x, y, angle, scale):
    angle_rad = math.radians(angle)
    x = CENTER[0] - float(x) + pan_x
    y = CENTER[1] - float(y) + pan_y

    x *= scale
    y *= scale

    x_new = (x * math.cos(angle_rad) - y * math.sin(angle_rad))
    y_new = (x * math.sin(angle_rad) + y * math.cos(angle_rad))
    return int(x_new + CENTER[0]), int(y_new + CENTER[1])

# Main loop
while running:

    screen.fill(WHITE)
    
    # Draw circular boundary
    pygame.draw.circle(screen, BLACK, CENTER, RADIUS, 3)

    title_text = font.render("F.E.I.N", True, BLACK)
    screen.blit(title_text, (WIDTH // 2 - title_text.get_width() // 2, 10))
    draw_axes()

    data, _ = sock.recvfrom(1024)
    data = data.decode("utf-8")

    if not data.startswith("Help!"):

        data = data.split(",")
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
                elif event.key == pygame.K_a:
                    pan_x += PAN_SPEED
                elif event.key == pygame.K_d:
                    pan_x -= PAN_SPEED
                elif event.key == pygame.K_w:
                    pan_y += PAN_SPEED
                elif event.key == pygame.K_s:
                    pan_y -= PAN_SPEED
                elif event.key == pygame.K_SPACE:
                    # Add a random point within the circle for testing
                    new_x = CENTER[0] + (math.cos(math.radians(angle_offset)) * (RADIUS // 2))
                    new_y = CENTER[1] + (math.sin(math.radians(angle_offset)) * (RADIUS // 2))
                    points.append((new_x, new_y, 0))
                elif event.key == pygame.K_UP:
                    scale_factor *= 1.1  # Zoom in (increase scale)
                elif event.key == pygame.K_DOWN:
                    scale_factor /= 1.1  # Zoom out (decrease scale)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        panning = True
                        mouse_start = event.pos
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        panning = False
                elif event.type == pygame.MOUSEMOTION and panning:
                    dx, dy = event.pos[0] - mouse_start[0], event.pos[1] - mouse_start[1]
                    pan_x += dx
                    pan_y += dy
                    mouse_start = event.pos
        
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
