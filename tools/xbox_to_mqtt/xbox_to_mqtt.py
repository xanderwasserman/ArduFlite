#!/usr/bin/env python3
"""
xbox_to_mqtt.py

Read an Xbox One controller via USB and re-publish its sticks as either:
  - Attitude setpoints (in degrees) → "arduflite/control/attitude"
  - CRSF-style 11-bit PWM values [172…1811] → "arduflite/receiver/emulator"

Usage:
    python xbox_to_mqtt.py --mode degrees
    python xbox_to_mqtt.py --mode pwm

Dependencies:
    pip install pygame paho-mqtt
"""

import sys
import time
import json
import argparse

import pygame
import paho.mqtt.client as mqtt

# ------------------------------------------------------------------------------------------------
# 1) Argument parsing (no changes here)
# ------------------------------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Xbox→MQTT bridge for ArduFlite")
parser.add_argument(
    "--mode",
    choices=["degrees", "crsf"],
    default="degrees",
    help="Publish format: 'degrees' (°) or 'crsf' (CRSF 11-bit)."
)
parser.add_argument(
    "--broker",
    default="bananapi",
    help="MQTT broker IP or hostname"
)
parser.add_argument(
    "--port",
    type=int,
    default=1883,
    help="MQTT broker port"
)
parser.add_argument(
    "--rate",
    type=int,
    default=20,
    help="Publish rate in Hz"
)
parser.add_argument(
    "--maxdeg",
    type=float,
    default=30.0,
    help="Maximum angle in degrees (±) when in 'degrees' mode"
)
args = parser.parse_args()

MODE        = args.mode
MQTT_BROKER = args.broker
MQTT_PORT   = args.port
PUBLISH_HZ  = args.rate
MAX_DEG     = args.maxdeg

if MODE == "degrees":
    MQTT_TOPIC = "arduflite/controller-setpoint/attitude/set"
else:
    MQTT_TOPIC = "arduflite/receiver/emulator/set"

print(f"[INFO] mode = {MODE}")
print(f"[INFO] publishing to '{MQTT_TOPIC}' at {PUBLISH_HZ} Hz")
if MODE == "degrees":
    print(f"[INFO] maxdeg = ±{MAX_DEG}°")
else:
    print(f"[INFO] CRSF-PWM range = [172…1811]")

# ------------------------------------------------------------------------------------------------
# 2) Initialize Pygame joystick (no changes)
# ------------------------------------------------------------------------------------------------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() < 1:
    print("Error: No joystick/gamepad detected.")
    sys.exit(1)

joy = pygame.joystick.Joystick(0)
joy.init()
print(f"[INFO] Using joystick: {joy.get_name()}")

# ------------------------------------------------------------------------------------------------
# 3) Initialize MQTT (no changes)
# ------------------------------------------------------------------------------------------------
client = mqtt.Client(client_id="xbox_bridge", clean_session=True)
try:
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
except Exception as e:
    print(f"Error: could not connect to MQTT broker at {MQTT_BROKER}:{MQTT_PORT} → {e}")
    sys.exit(1)

client.loop_start()

# ------------------------------------------------------------------------------------------------
# 4) Helper functions
# ------------------------------------------------------------------------------------------------
def apply_deadband(value, db=0.05):
    """Zero-out small stick noise within ±db."""
    return 0.0 if abs(value) < db else value

def axis_to_degrees(raw, max_deg=MAX_DEG):
    """Map raw axis [-1.0…+1.0] → [-max_deg…+max_deg] degrees."""
    return float(round(raw * max_deg, 3))

def axis_to_crsf_pwm(raw):
    """
    Map raw axis [-1.0…+1.0] → [172…1811] (CRSF 11-bit range).
    Formula: pwm = (raw + 1)/2 * (1811 - 172) + 172
    """
    # Range = 1811 - 172 = 1639
    mapped = (raw + 1.0) * 0.5 * 1639.0 + 172.0
    return int(round(mapped))

# ------------------------------------------------------------------------------------------------
# 5) Main loop: read sticks and publish JSON
# ------------------------------------------------------------------------------------------------
try:
    interval = 1.0 / PUBLISH_HZ
    print(f"[INFO] Starting publish loop (Ctrl-C to quit)…")

    while True:
        pygame.event.pump()

        # Axis mapping:
        raw_roll   = joy.get_axis(2)  # right stick X
        raw_pitch = joy.get_axis(3)  # right stick Y
        raw_yaw  = joy.get_axis(0)  # left stick X
        

        x_roll  = apply_deadband(raw_roll)
        x_pitch = apply_deadband(raw_pitch)
        x_yaw   = apply_deadband(raw_yaw)

        if MODE == "degrees":
            # Invert so “stick right” = positive roll, “stick up” = positive pitch
            roll_val  = axis_to_degrees(x_roll)
            pitch_val = axis_to_degrees(x_pitch)
            yaw_val   = axis_to_degrees(x_yaw)

            payload = json.dumps({
                "roll":  roll_val,
                "pitch": pitch_val,
                "yaw":   yaw_val
            })

        else:  # MODE == "crsf"
            # CRSF-PWM mapping in [172…1811]
            pwm_roll  = axis_to_crsf_pwm(x_roll)
            pwm_pitch = axis_to_crsf_pwm(x_pitch)
            pwm_yaw   = axis_to_crsf_pwm(x_yaw)

            payload = json.dumps({
                "roll":  pwm_roll,
                "pitch": pwm_pitch,
                "yaw":   pwm_yaw
            })

        client.publish(MQTT_TOPIC, payload)
        time.sleep(interval)

except KeyboardInterrupt:
    print("\n[INFO] Exiting…")
finally:
    client.loop_stop()
    client.disconnect()
    pygame.quit()
    sys.exit(0)
