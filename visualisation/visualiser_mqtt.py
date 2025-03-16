import sys
import math
import pygame
import numpy as np
import paho.mqtt.client as mqtt

from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# Global quaternion values, initialized to a default (no rotation)
latest_w = 1.0
latest_x = 0.0
latest_y = 0.0
latest_z = 0.0

# -------------------------------------------------
# MQTT callbacks
# -------------------------------------------------
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("arduflite/quaternion/w")
    client.subscribe("arduflite/quaternion/x")
    client.subscribe("arduflite/quaternion/y")
    client.subscribe("arduflite/quaternion/z")

def on_message(client, userdata, msg):
    global latest_w, latest_x, latest_y, latest_z
    try:
        value = float(msg.payload.decode("utf-8").strip())
    except ValueError:
        return  # skip malformed values
    if msg.topic == "arduflite/quaternion/w":
        latest_w = value
    elif msg.topic == "arduflite/quaternion/x":
        latest_x = value
    elif msg.topic == "arduflite/quaternion/y":
        latest_y = value
    elif msg.topic == "arduflite/quaternion/z":
        latest_z = value

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
# Draw a filled aircraft model with metallic silver/grey.
# Also overlays a black wireframe edge.
# Coordinate system: x (forward), y (right), z (down)
# -------------------------------------------------
def draw_aircraft_filled():
    # Set the metallic color.
    glColor3f(0.75, 0.75, 0.75)
    
    # Parameters for fuselage cross-sections.
    xs = [-1.0, -0.5, 0.0, 0.5, 1.0]
    rys = [0.03, 0.05, 0.1, 0.1, 0.02]
    rzs = [0.03, 0.05, 0.1, 0.1, 0.02]
    num_segments = 16
    
    cross_sections = []
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
    
    # Fill fuselage using quad strips.
    for i in range(len(cross_sections) - 1):
        sec1 = cross_sections[i]
        sec2 = cross_sections[i + 1]
        glBegin(GL_QUAD_STRIP)
        for j in range(num_segments + 1):
            idx = j % num_segments
            glVertex3f(*sec1[idx])
            glVertex3f(*sec2[idx])
        glEnd()
    
    # Fill left wing.
    glBegin(GL_QUADS)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, -1.2, 0.0)
    glVertex3f(0.2, -1.2, 0.0)
    glVertex3f(0.2, 0.0, 0.0)
    glEnd()
    
    # Fill right wing.
    glBegin(GL_QUADS)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 1.2, 0.0)
    glVertex3f(0.2, 1.2, 0.0)
    glVertex3f(0.2, 0.0, 0.0)
    glEnd()
    
    # Fill horizontal stabilizer.
    glBegin(GL_QUADS)
    glVertex3f(-1.0, -0.4, 0.0)
    glVertex3f(-0.8, -0.4, 0.0)
    glVertex3f(-0.8, 0.4, 0.0)
    glVertex3f(-1.0, 0.4, 0.0)
    glEnd()
    
    # Fill vertical fin as a triangle.
    glBegin(GL_TRIANGLES)
    glVertex3f(-1.0, 0.0, 0.0)
    glVertex3f(-1.0, 0.0, -0.6)
    glVertex3f(-0.7, 0.0, 0.0)
    glEnd()

def draw_aircraft_wireframe_black():
    # Draw the same geometry in black for wireframe overlay.
    glColor3f(0, 0, 0)
    
    # Parameters for fuselage cross-sections.
    xs = [-1.0, -0.5, 0.0, 0.5, 1.0]
    rys = [0.03, 0.05, 0.1, 0.1, 0.02]
    rzs = [0.03, 0.05, 0.1, 0.1, 0.02]
    num_segments = 16
    
    cross_sections = []
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
    
    # Draw fuselage wireframe.
    for section in cross_sections:
        glBegin(GL_LINE_LOOP)
        for pt in section:
            glVertex3f(*pt)
        glEnd()
    for i in range(len(cross_sections) - 1):
        sec1 = cross_sections[i]
        sec2 = cross_sections[i+1]
        glBegin(GL_LINES)
        for j in range(num_segments):
            glVertex3f(*sec1[j])
            glVertex3f(*sec2[j])
        glEnd()
    
    # Left wing wireframe.
    glBegin(GL_LINE_LOOP)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, -1.2, 0.0)
    glVertex3f(0.2, -1.2, 0.0)
    glVertex3f(0.2, 0.0, 0.0)
    glEnd()
    
    # Right wing wireframe.
    glBegin(GL_LINE_LOOP)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 1.2, 0.0)
    glVertex3f(0.2, 1.2, 0.0)
    glVertex3f(0.2, 0.0, 0.0)
    glEnd()
    
    # Horizontal stabilizer wireframe.
    glBegin(GL_LINE_LOOP)
    glVertex3f(-1.0, -0.4, 0.0)
    glVertex3f(-0.8, -0.4, 0.0)
    glVertex3f(-0.8, 0.4, 0.0)
    glVertex3f(-1.0, 0.4, 0.0)
    glEnd()
    
    # Vertical fin wireframe.
    glBegin(GL_LINE_LOOP)
    glVertex3f(-1.0, 0.0, 0.0)
    glVertex3f(-1.0, 0.0, -0.6)
    glVertex3f(-0.7, 0.0, 0.0)
    glEnd()

# -------------------------------------------------
# Draw a solid filled ground plane in the x-y plane at z=0.
# -------------------------------------------------
def draw_ground():
    glColor3f(0.25, 0.54, 0.33)
    glBegin(GL_QUADS)
    glVertex3f(-10, -10, 0)
    glVertex3f(-10,  10, 0)
    glVertex3f( 10,  10, 0)
    glVertex3f( 10, -10, 0)
    glEnd()

# -------------------------------------------------
# Main function: sets up MQTT communication, initializes Pygame & OpenGL,
# and renders the scene.
# -------------------------------------------------
def main():
    global latest_w, latest_x, latest_y, latest_z

    broker_address = "192.168.100.14"
    broker_port = 1883

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(broker_address, broker_port, 60)
    except Exception as e:
        print("Error connecting to MQTT broker:", e)
        sys.exit(1)
    client.loop_start()

    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Aircraft Orientation with MQTT & Metallic Aircraft")

    glClearColor(0.53, 0.81, 0.92, 1.0)

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

        roll, pitch, yaw = quaternion_to_euler(latest_w, latest_x, latest_y, latest_z)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        # Position the camera behind the aircraft along -x.
        gluLookAt(-5, 0, -1.5, 0, 0, -1.5, 0, 0, -10)

        draw_ground()

        glPushMatrix()
        # Raise the aircraft further above the ground.
        glTranslatef(0, 0, -1.5)
        glRotatef(yaw_deg, 0, 0, 1)
        glRotatef(pitch_deg, 0, -1, 0)
        glRotatef(roll_deg, 1, 0, 0)
        # Draw filled metallic aircraft.
        draw_aircraft_filled()
        # Overlay black wireframe.
        glEnable(GL_POLYGON_OFFSET_LINE)
        glPolygonOffset(-2.0, -2.0)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        draw_aircraft_wireframe_black()
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glDisable(GL_POLYGON_OFFSET_LINE)
        glPopMatrix()

        pygame.display.flip()
        clock.tick(60)

    client.loop_stop()
    client.disconnect()
    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    main()
