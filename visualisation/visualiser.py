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
# Draw an improved 3D wireframe aircraft model.
# Coordinate system: x (forward), y (right), z (down)
#
# The fuselage is built using several elliptical cross-sections
# along the x-axis (from tail to nose). The wings, horizontal
# stabilizer, and vertical fin (fin extends in -z) are added.
# -------------------------------------------------
def draw_aircraft_wireframe():
    glColor3f(1, 1, 1)  # white color for the aircraft

    # Parameters for fuselage cross-sections:
    xs = [-1.0, -0.5, 0.0, 0.5, 1.0]       # x positions (tail to nose)
    rys = [0.03, 0.05, 0.1, 0.1, 0.02]       # half-width in y at each section
    rzs = [0.03, 0.05, 0.1, 0.1, 0.02]       # half-height in z at each section
    num_segments = 16                       # number of segments for each ellipse

    cross_sections = []  # will hold lists of (x,y,z) points for each cross-section

    # Generate elliptical cross-sections in the y-z plane for each x.
    for i, x in enumerate(xs):
        section = []
        r_y = rys[i]
        r_z = rzs[i]
        for j in range(num_segments):
            angle = 2 * math.pi * j / num_segments
            y = r_y * math.cos(angle)
            z = r_z * math.sin(angle)
            section.append((x, y, z))
        cross_sections.append(section)

    # Draw each cross-section as a line loop.
    for section in cross_sections:
        glBegin(GL_LINE_LOOP)
        for pt in section:
            glVertex3f(*pt)
        glEnd()

    # Connect corresponding points between adjacent cross-sections.
    for i in range(len(cross_sections) - 1):
        sec1 = cross_sections[i]
        sec2 = cross_sections[i+1]
        glBegin(GL_LINES)
        for j in range(num_segments):
            glVertex3f(*sec1[j])
            glVertex3f(*sec2[j])
        glEnd()

    # ---- Add wings ----
    # Wings attach roughly at mid-fuselage (x ~ 0).
    glColor3f(1, 1, 1)
    # Left wing (extends toward negative y)
    glBegin(GL_LINE_LOOP)
    glVertex3f(0.0, 0.0, 0.0)       # attachment at fuselage center
    glVertex3f(0.0, -1.2, 0.0)      # wing tip
    glVertex3f(0.2, -1.2, 0.0)      # chord thickness
    glVertex3f(0.2, 0.0, 0.0)       # back to fuselage
    glEnd()
    # Right wing (extends toward positive y)
    glBegin(GL_LINE_LOOP)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 1.2, 0.0)
    glVertex3f(0.2, 1.2, 0.0)
    glVertex3f(0.2, 0.0, 0.0)
    glEnd()

    # ---- Horizontal Stabilizer (tail) ----
    glBegin(GL_LINE_LOOP)
    glVertex3f(-1.0, -0.4, 0.0)
    glVertex3f(-0.8, -0.4, 0.0)
    glVertex3f(-0.8, 0.4, 0.0)
    glVertex3f(-1.0, 0.4, 0.0)
    glEnd()

    # ---- Vertical Fin (tail) ----
    # Note: The fin extends in the -z direction (tail is fixed to -z).
    glBegin(GL_LINE_LOOP)
    glVertex3f(-1.0, 0.0, 0.0)     # base at fuselage
    glVertex3f(-1.0, 0.0, -0.6)    # tip (extending in -z)
    glVertex3f(-0.7, 0.0, 0.0)     # other base point
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
    glColor3f(0.3, 0.3, 0.3)  # dark gray for the grid
    glBegin(GL_LINES)
    grid_size = 10
    spacing = 0.5  # adjust grid spacing as desired
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
#
# The serial reading section now reads all available lines,
# discarding older data so that only the latest reading is used.
# This ensures the update is independent of the controller's frequency.
# -------------------------------------------------
def main():
    # 1) Set up your serial device
    PORT = "COM4"       # Adjust as needed (e.g., "COM3" or "/dev/ttyUSB0")
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
    pygame.display.set_caption("Enhanced Aircraft Orientation with 3D Ground")
    
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

        # --- Read all available serial data and use the latest line ---
        line = None
        while ser.inWaiting() > 0:
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
            # is in front. The up vector is set to (0,0,-1) because "up" is opposite to down.
            gluLookAt(-5, 0, -2,   0, 0, 0,   0, 0, -1)

            # Draw the fixed ground grid (world space) in the x-y plane (z=0).
            glPushMatrix()
            draw_grid()
            glPopMatrix()

            # Optionally, draw fixed coordinate axes.
            glPushMatrix()
            # draw_axes()  # Uncomment to see axes.
            glPopMatrix()

            # Draw the aircraft.
            # Apply rotations in the order:
            #   yaw (about z), pitch (about y), then roll (about x).
            glPushMatrix()
            glRotatef(yaw_deg, 0, 0, 1)    # yaw about z (down)
            glRotatef(pitch_deg, 0, 1, 0)   # pitch about y (right)
            glRotatef(roll_deg, 1, 0, 0)    # roll about x (forward)
            draw_aircraft_wireframe()
            glPopMatrix()

            pygame.display.flip()
        clock.tick(60)

    ser.close()
    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    main()
