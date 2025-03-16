import sys
import math
import serial
import pygame
import numpy as np

from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# -------------------------------------------------
# Helper function to convert a quaternion to Euler.
# Euler angles in this order: roll (X), pitch (Y), yaw (Z).
# -------------------------------------------------
def quaternion_to_euler(w, x, y, z):
    """
    Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw).
    Returns angles in radians.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)

# -------------------------------------------------
# Draw a simple wireframe aircraft shape.
# We'll draw a rough shape with a nose, tail, wings.
# -------------------------------------------------
def draw_aircraft_wireframe():
    glBegin(GL_LINES)

    # Fuselage
    glVertex3f(0.0, 0.0, 0.5)   # Nose
    glVertex3f(0.0, 0.0, -0.5)  # Tail

    # Wings
    glVertex3f(-0.5, 0.0, 0.0)
    glVertex3f(0.5, 0.0, 0.0)

    # Vertical stabilizer
    glVertex3f(0.0, 0.0, -0.5)
    glVertex3f(0.0, 0.3, -0.5)

    glEnd()

# -------------------------------------------------
# Main function:
#  - Initialize Pygame & OpenGL
#  - Open serial port
#  - Read and parse quaternions
#  - Convert to orientation & display
# -------------------------------------------------
def main():
    # ------------------------------
    # 1) Set up your serial device
    # Adjust PORT and BAUDRATE as needed.
    # ------------------------------
    PORT = "COM4"       # e.g. "COM3" on Windows or "/dev/ttyUSB0" on Linux
    BAUDRATE = 115200    # Ensure that matches your microcontroller
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Opened serial port: {PORT} at {BAUDRATE}")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # ------------------------------
    # 2) Initialize Pygame and create an OpenGL-capable window
    # ------------------------------
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Real-time Aircraft Orientation")

    # ------------------------------
    # 3) Setup basic OpenGL perspective and other settings
    # ------------------------------
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)

    glEnable(GL_DEPTH_TEST)
    glTranslatef(0.0, 0.0, -3.0)  # Move back so we can see the aircraft

    clock = pygame.time.Clock()

    # ------------------------------
    # Main loop
    # ------------------------------
    running = True
    while running:
        # Handle pygame events (e.g. close window)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 3a) Read serial line for quaternion data
        #     Expected format: "qw,qx,qy,qz"
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                qw, qx, qy, qz = map(float, line.split(','))
            except ValueError:
                # If there's a parsing error, skip this line
                continue

            # 3b) Convert quaternion to Euler angles (radians)
            roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)

            # 3c) Clear screen & depth buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            # Reset transformations
            glLoadIdentity()
            glTranslatef(0.0, 0.0, -3.0)

            # Convert from radians to degrees for glRotatef
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)

            # Adjust yaw by 180° and invert the x-axis rotation (pitch)
            glRotatef(yaw_deg +180, 0,-1, 0)   # Yaw: add 180°
            glRotatef(pitch_deg, -1, 0, 0)       # Pitch: invert x-axis (same as using -pitch_deg with a positive axis)
            glRotatef(roll_deg, 0, 0, 1)         # Roll remains the same
            # 3d) Draw the aircraft wireframe
            draw_aircraft_wireframe()

            # Swap buffers to update the display
            pygame.display.flip()

        # Limit to ~50 fps (matching ~50 Hz from microcontroller)
        clock.tick(50)

    # ------------------------------
    # Cleanup
    # ------------------------------
    ser.close()
    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    main()
