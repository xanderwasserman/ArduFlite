import sys
import math
import serial
import pygame
import numpy as np

from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# -------------------------------------------------
# Convert a quaternion to Euler angles (roll, pitch, yaw).
# Our aerospace frame now uses:
#   Roll:  rotation about x-axis (forward)
#   Pitch: rotation about y-axis (right)
#   Yaw:   rotation about z-axis (down)
# -------------------------------------------------
def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# -------------------------------------------------
# Draw a simple wireframe aircraft in our aerospace
# coordinate system (no flips):
#   x-axis: front (nose at +0.5)
#   y-axis: right (wings extend along ±y)
#   z-axis: down (vertical stabilizer extends down)
# -------------------------------------------------
def draw_aircraft_wireframe():
    glColor3f(1, 1, 1)  # white
    glBegin(GL_LINES)
    # Fuselage: from tail to nose along x-axis
    glVertex3f(-0.5, 0, 0)
    glVertex3f(0.5, 0, 0)
    # Wings: extend left (-y) to right (+y)
    glVertex3f(0, -0.5, 0)
    glVertex3f(0, 0.5, 0)
    # Vertical stabilizer: at tail, extending downward (positive z)
    glVertex3f(-0.5, 0, 0)
    glVertex3f(-0.5, 0, -0.3)
    glEnd()

# -------------------------------------------------
# Draw fixed coordinate axes at the world origin.
# According to our desired system:
#   X (red): forward
#   Y (green): right
#   Z (blue): down
# -------------------------------------------------
def draw_axes():
    glBegin(GL_LINES)
    # X axis (red): forward
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(1, 0, 0)
    # Y axis (green): right
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 1, 0)
    # Z axis (blue): down
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 1)
    glEnd()

# -------------------------------------------------
# Draw a ground grid in the horizontal (x-y) plane at z=0.
# With perspective, the grid recedes toward the horizon.
# -------------------------------------------------
def draw_grid():
    glColor3f(0.3, 0.3, 0.3)  # dark gray
    glBegin(GL_LINES)
    grid_size = 10
    spacing = 0.5  # adjust grid spacing as needed
    for i in range(-grid_size, grid_size + 1):
        # Vertical grid lines (constant x)
        glVertex3f(i * spacing, -grid_size * spacing, 0)
        glVertex3f(i * spacing, grid_size * spacing, 0)
        # Horizontal grid lines (constant y)
        glVertex3f(-grid_size * spacing, i * spacing, 0)
        glVertex3f(grid_size * spacing, i * spacing, 0)
    glEnd()

# -------------------------------------------------
# Main function: sets up serial communication,
# initializes Pygame & OpenGL, and renders the scene.
# -------------------------------------------------
def main():
    # 1) Set up your serial device
    PORT = "COM4"       # Adjust as needed (e.g., "COM3" on Windows or "/dev/ttyUSB0" on Linux)
    BAUDRATE = 115200   # Must match your microcontroller settings
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Opened serial port: {PORT} at {BAUDRATE}")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # 2) Initialize Pygame and create an OpenGL-capable window
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Aircraft Orientation with 3D Ground")
    
    # 3) Set up perspective projection.
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)

    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Read serial data (expected format: "qw,qx,qy,qz")
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                qw, qx, qy, qz = map(float, line.split(','))
            except ValueError:
                continue  # skip malformed lines

            # Convert quaternion to Euler angles (radians)
            roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)

            # Clear screen and set up the camera.
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            # Position the camera behind the aircraft along -x.
            # With our desired system (x: forward, y: right, z: down),
            # place the camera at (-5, 0, -2) so that the aircraft (at the origin)
            # is in front. The up vector is set to (0,0,-1) because in our system,
            # "up" is opposite to down.
            gluLookAt(-5, 0, -2,   0, 0, 0,   0, 0, -1)

            # -------------------------------------------------
            # Draw the fixed ground grid (world space) in the x-y plane (z=0).
            # -------------------------------------------------
            glPushMatrix()
            draw_grid()
            glPopMatrix()

            # Optionally, draw fixed coordinate axes.
            glPushMatrix()
            # draw_axes()
            glPopMatrix()

            # -------------------------------------------------
            # Draw the aircraft.
            # Apply rotations in the order:
            #   yaw (about z), pitch (about y), then roll (about x).
            # This assumes our quaternion-to-Euler conversion matches our aerospace convention.
            # No additional scaling is applied since our aircraft model is already defined with:
            #   x: forward, y: right, z: down.
            # -------------------------------------------------
            glPushMatrix()
            glRotatef(yaw_deg, 0, 0, 1)    # yaw about z (down)
            glRotatef(pitch_deg, 0, 1, 0)  # pitch about y (right)
            glRotatef(roll_deg, 1, 0, 0)   # roll about x (forward)
            draw_aircraft_wireframe()
            glPopMatrix()

            pygame.display.flip()
        clock.tick(50)

    ser.close()
    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    main()
