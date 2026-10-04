/**
 * ConfigSchema.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Registration of all configuration parameters.
 *        Include this file ONCE in a .cpp to trigger static registration.
 *        Uses macros from ConfigRegistry.h with keys from ConfigKeys.h.
 */
#ifndef CONFIG_SCHEMA_H
#define CONFIG_SCHEMA_H

#include "include/ConfigKeys.h"
#include "src/utils/ConfigRegistry.h"

// ═══════════════════════════════════════════════════════════════════════════
// This file is designed to be included in exactly ONE .cpp file
// (typically ConfigSchema.cpp or ArdufliteApp.cpp)
// ═══════════════════════════════════════════════════════════════════════════

#ifdef CONFIG_SCHEMA_IMPL

// ═══════════════════════════════════════════════════════════════════════════
// Rate Controller PID (Inner Loop)
// ═══════════════════════════════════════════════════════════════════════════

// Roll
CONFIG_FLOAT(CONFIG_KEY_RATE_ROLL_KP,       0.09f,  0.0f,   1.0f,   "Roll rate P gain");
CONFIG_FLOAT(CONFIG_KEY_RATE_ROLL_TI_S,       1.40f,  0.0f,   10.0f,  "Roll rate I time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_RATE_ROLL_TD_S,       0.30f,  0.0f,   1.0f,   "Roll rate D time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_RATE_ROLL_OUTLIMIT, 1.00f,  0.1f,   1.0f,   "Roll rate output limit");
CONFIG_FLOAT(CONFIG_KEY_RATE_ROLL_HEADROOM, 0.80f,  0.5f,   1.0f,   "Roll rate anti-windup headroom");
CONFIG_FLOAT(CONFIG_KEY_RATE_ROLL_ALPHA,    0.10f,  0.01f,  1.0f,   "Roll rate D filter alpha");

// Pitch
CONFIG_FLOAT(CONFIG_KEY_RATE_PITCH_KP,       0.04f,  0.0f,   1.0f,   "Pitch rate P gain");
CONFIG_FLOAT(CONFIG_KEY_RATE_PITCH_TI_S,       5.00f,  0.0f,   10.0f,  "Pitch rate I time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_RATE_PITCH_TD_S,       0.45f,  0.0f,   1.0f,   "Pitch rate D time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_RATE_PITCH_OUTLIMIT, 1.00f,  0.1f,   1.0f,   "Pitch rate output limit");
CONFIG_FLOAT(CONFIG_KEY_RATE_PITCH_HEADROOM, 0.80f,  0.5f,   1.0f,   "Pitch rate anti-windup headroom");
CONFIG_FLOAT(CONFIG_KEY_RATE_PITCH_ALPHA,    0.10f,  0.01f,  1.0f,   "Pitch rate D filter alpha");

// Yaw
CONFIG_FLOAT(CONFIG_KEY_RATE_YAW_KP,       0.05f,  0.0f,   1.0f,   "Yaw rate P gain");
CONFIG_FLOAT(CONFIG_KEY_RATE_YAW_TI_S,       8.0f,   0.0f,   10.0f,  "Yaw rate I time constant (s) - slow by design; see ADR-054. 0 disables");
CONFIG_FLOAT(CONFIG_KEY_RATE_YAW_TD_S,       0.30f,  0.0f,   1.0f,   "Yaw rate D time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_RATE_YAW_OUTLIMIT, 1.00f,  0.1f,   1.0f,   "Yaw rate output limit");
CONFIG_FLOAT(CONFIG_KEY_RATE_YAW_HEADROOM, 0.80f,  0.5f,   1.0f,   "Yaw rate anti-windup headroom");
CONFIG_FLOAT(CONFIG_KEY_RATE_YAW_ALPHA,    0.10f,  0.01f,  1.0f,   "Yaw rate D filter alpha");

// Rate output filter
CONFIG_FLOAT(CONFIG_KEY_RATE_OUT_LP_ALPHA, 0.3f,   0.001f, 1.0f,   "Rate output low-pass filter alpha");

// ═══════════════════════════════════════════════════════════════════════════
// Attitude Controller PID (Outer Loop)
// ═══════════════════════════════════════════════════════════════════════════

// Roll
CONFIG_FLOAT(CONFIG_KEY_ATT_ROLL_KP,       320.0f, 0.0f,   1000.0f, "Roll attitude P gain");
CONFIG_FLOAT(CONFIG_KEY_ATT_ROLL_TI_S,       0.00f,  0.0f,   10.0f,   "Roll attitude I time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_ROLL_TD_S,       0.00f,  0.0f,   1.0f,    "Roll attitude D time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_ROLL_OUTLIMIT_DPS, 90.0f,  10.0f,  180.0f,  "Roll attitude output limit (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_ROLL_HEADROOM, 0.80f,  0.5f,   1.0f,    "Roll attitude anti-windup headroom");
CONFIG_FLOAT(CONFIG_KEY_ATT_ROLL_ALPHA,    0.10f,  0.01f,  1.0f,    "Roll attitude D filter alpha");

// Pitch
CONFIG_FLOAT(CONFIG_KEY_ATT_PITCH_KP,       150.0f, 0.0f,   1000.0f, "Pitch attitude P gain");
CONFIG_FLOAT(CONFIG_KEY_ATT_PITCH_TI_S,       0.00f,  0.0f,   10.0f,   "Pitch attitude I time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_PITCH_TD_S,       0.00f,  0.0f,   1.0f,    "Pitch attitude D time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_PITCH_OUTLIMIT_DPS, 45.0f,  10.0f,  180.0f,  "Pitch attitude output limit (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_PITCH_HEADROOM, 0.80f,  0.5f,   1.0f,    "Pitch attitude anti-windup headroom");
CONFIG_FLOAT(CONFIG_KEY_ATT_PITCH_ALPHA,    0.10f,  0.01f,  1.0f,    "Pitch attitude D filter alpha");

// Yaw
CONFIG_FLOAT(CONFIG_KEY_ATT_YAW_KP,       200.0f, 0.0f,   1000.0f, "Yaw attitude P gain");
CONFIG_FLOAT(CONFIG_KEY_ATT_YAW_TI_S,       0.00f,  0.0f,   10.0f,   "Yaw attitude I time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_YAW_TD_S,       0.00f,  0.0f,   1.0f,    "Yaw attitude D time constant (s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_YAW_OUTLIMIT_DPS, 60.0f,  10.0f,  180.0f,  "Yaw attitude output limit (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_ATT_YAW_HEADROOM, 0.80f,  0.5f,   1.0f,    "Yaw attitude anti-windup headroom");
CONFIG_FLOAT(CONFIG_KEY_ATT_YAW_ALPHA,    0.10f,  0.01f,  1.0f,    "Yaw attitude D filter alpha");

// Attitude deadband
CONFIG_FLOAT(CONFIG_KEY_ATT_DEADBAND_RAD, 0.0001f, 0.0f, 0.01f, "Attitude error deadband (radians)");

// ═══════════════════════════════════════════════════════════════════════════
// Mixer Configuration
// ═══════════════════════════════════════════════════════════════════════════

CONFIG_FLOAT(CONFIG_KEY_MIX_MAX_ATT_ROLL_DEG,    55.0f,  10.0f, 90.0f,  "Max roll angle in attitude mode (deg)");
CONFIG_FLOAT(CONFIG_KEY_MIX_MAX_ATT_PITCH_DEG,   50.0f,  10.0f, 90.0f,  "Max pitch angle in attitude mode (deg)");
CONFIG_FLOAT(CONFIG_KEY_MIX_MAX_ATT_YAW_DEG,     180.0f, 45.0f, 360.0f, "Max yaw heading offset (deg)");
CONFIG_FLOAT(CONFIG_KEY_MIX_MAX_RATE_ROLL_DPS,   90.0f,  30.0f, 360.0f, "Max roll rate in rate mode (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_MIX_MAX_RATE_PITCH_DPS,  60.0f,  30.0f, 360.0f, "Max pitch rate in rate mode (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_MIX_MAX_RATE_YAW_DPS,    60.0f,  30.0f, 360.0f, "Max yaw rate in rate mode (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_MIX_ROLL_FROM_YAW,   0.00f,  0.0f,  0.5f,   "SAFE yaw->roll mixing coefficient");
CONFIG_FLOAT(CONFIG_KEY_MIX_PITCH_FROM_ROLL, 0.08f,  0.0f,  0.5f,   "SAFE roll->pitch mixing coefficient");
CONFIG_FLOAT(CONFIG_KEY_MIX_YAW_FROM_ROLL,   0.10f,  0.0f,  0.5f,   "SAFE roll->yaw mixing coefficient");

// ═══════════════════════════════════════════════════════════════════════════
// Servo Configuration
// ═══════════════════════════════════════════════════════════════════════════

// Wing/airframe geometry (0=CONVENTIONAL, 1=DELTA_WING, 2=V_TAIL)
CONFIG_INT(CONFIG_KEY_SERVO_WING_DESIGN,   0,     0,    2,    "Wing design (0=Conventional, 1=Delta, 2=V-Tail)");
CONFIG_BOOL(CONFIG_KEY_SERVO_DUAL_AILERONS, true,       "Use dual ailerons (vs single)");

CONFIG_FLOAT(CONFIG_KEY_SERVO_MAX_SLEW_DPS, 500.0f, 100.0f, 1000.0f, "Max servo slew rate (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_SERVO_MAX_THR_SLEW_PER_S, 1.0f,   0.1f,   5.0f,    "Max throttle slew rate (range/s)");

// Pitch servo
CONFIG_INT(CONFIG_KEY_SERVO_PITCH_MIN_US,      500,   500,  1000, "Pitch servo min pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_PITCH_MAX_US,      2500,  2000, 2500, "Pitch servo max pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_PITCH_NEUTRAL_DEG,  90,    0,    180,  "Pitch servo neutral angle (deg)");
CONFIG_INT(CONFIG_KEY_SERVO_PITCH_DEFL_DEG,     80,    10,   90,   "Pitch servo max deflection (deg)");
CONFIG_BOOL(CONFIG_KEY_SERVO_PITCH_INV,     true,        "Pitch servo invert direction");

// Yaw servo
CONFIG_INT(CONFIG_KEY_SERVO_YAW_MIN_US,      500,   500,  1000, "Yaw servo min pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_YAW_MAX_US,      2500,  2000, 2500, "Yaw servo max pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_YAW_NEUTRAL_DEG,  90,    0,    180,  "Yaw servo neutral angle (deg)");
CONFIG_INT(CONFIG_KEY_SERVO_YAW_DEFL_DEG,     80,    10,   90,   "Yaw servo max deflection (deg)");
CONFIG_BOOL(CONFIG_KEY_SERVO_YAW_INV,     false,       "Yaw servo invert direction");

// Left aileron
CONFIG_INT(CONFIG_KEY_SERVO_LAIL_MIN_US,      500,   500,  1000, "Left aileron min pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_LAIL_MAX_US,      2500,  2000, 2500, "Left aileron max pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_LAIL_NEUTRAL_DEG,  90,    0,    180,  "Left aileron neutral angle (deg)");
CONFIG_INT(CONFIG_KEY_SERVO_LAIL_DEFL_DEG,     80,    10,   90,   "Left aileron max deflection (deg)");
CONFIG_BOOL(CONFIG_KEY_SERVO_LAIL_INV,     true,        "Left aileron invert direction");

// Right aileron
CONFIG_INT(CONFIG_KEY_SERVO_RAIL_MIN_US,      500,   500,  1000, "Right aileron min pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_RAIL_MAX_US,      2500,  2000, 2500, "Right aileron max pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_RAIL_NEUTRAL_DEG,  90,    0,    180,  "Right aileron neutral angle (deg)");
CONFIG_INT(CONFIG_KEY_SERVO_RAIL_DEFL_DEG,     80,    10,   90,   "Right aileron max deflection (deg)");
CONFIG_BOOL(CONFIG_KEY_SERVO_RAIL_INV,     false,       "Right aileron invert direction");

// Throttle
CONFIG_INT(CONFIG_KEY_SERVO_THR_MIN_US,  1000, 500,  1500, "Throttle min pulse (us)");
CONFIG_INT(CONFIG_KEY_SERVO_THR_MAX_US,  2000, 1500, 2500, "Throttle max pulse (us)");

// ═══════════════════════════════════════════════════════════════════════════
// IMU Configuration
// ═══════════════════════════════════════════════════════════════════════════

CONFIG_FLOAT(CONFIG_KEY_IMU_ACCEL_ALPHA,    0.02f,  0.001f, 1.0f,   "Accelerometer low-pass filter alpha");
CONFIG_FLOAT(CONFIG_KEY_IMU_GYRO_ALPHA,     0.4f,   0.001f, 1.0f,   "Gyroscope low-pass filter alpha");
CONFIG_FLOAT(CONFIG_KEY_IMU_MAG_ALPHA,      0.04f,  0.001f, 1.0f,   "Magnetometer low-pass filter alpha");
CONFIG_FLOAT(CONFIG_KEY_IMU_ALTI_ALPHA,     0.10f,  0.001f, 0.50f, "Altimeter low-pass filter alpha (at 50 Hz baro rate; 0.10 → τ≈200ms)");
CONFIG_FLOAT(CONFIG_KEY_IMU_MADGWICK_BETA,  0.1f,   0.01f, 1.0f,   "Madgwick filter beta (gyro/accel trust)");
// Off by design, not pending. A magnetometer that is FITTED is a board fact;
// whether it is TRUSTED in the fusion loop is a separate judgement, and today
// the answer is no - nothing consumes heading, while an uncalibrated field
// steals accelerometer authority from roll and pitch. See ADR-055.
CONFIG_BOOL(CONFIG_KEY_IMU_FUSE_MAG,       false,          "Fuse magnetometer into attitude (9-axis). Needs hard-iron calibration first");
CONFIG_FLOAT(CONFIG_KEY_IMU_MAX_ACCEL_G,    16.0f,  4.0f,  16.0f,  "Max valid accelerometer magnitude (g)");
CONFIG_FLOAT(CONFIG_KEY_IMU_MAX_GYRO_DPS,   2000.0f, 250.0f, 2000.0f, "Max valid gyroscope rate (deg/s)");
CONFIG_UINT8(CONFIG_KEY_IMU_FAIL_THRESHOLD, 5,      1,     20,     "Consecutive failures before unhealthy");
CONFIG_FLOAT(CONFIG_KEY_IMU_GYRO_BIAS_MAX_DPS,  5.0f,   1.0f,  20.0f,  "Max acceptable gyro bias (deg/s)");
CONFIG_FLOAT(CONFIG_KEY_IMU_EXPECTED_G,     1.0f,   0.9f,  1.1f,   "Expected gravity magnitude (g)");
CONFIG_FLOAT(CONFIG_KEY_IMU_GRAVITY_TOL_G,    0.15f,  0.05f, 0.3f,   "Gravity reading tolerance (g)");
CONFIG_FLOAT(CONFIG_KEY_IMU_LAUNCH_ACCEL_G,   0.10f,  0.05f, 2.0f,   "Launch: accel deviation from 1 g (g; reboot to apply)");
CONFIG_FLOAT(CONFIG_KEY_IMU_LAUNCH_GYRO_DPS,  15.0f,  5.0f,  149.0f, "Launch: rotation rate (deg/s; reboot to apply)");
CONFIG_INT(CONFIG_KEY_IMU_LAUNCH_MS,          50,     10,    1000,   "Launch: how long the motion must last (ms; reboot to apply)");

// ═══════════════════════════════════════════════════════════════════════════
// Failsafe Configuration
// ═══════════════════════════════════════════════════════════════════════════

CONFIG_FLOAT(CONFIG_KEY_FS_BANK_DEG,   7.0f,  0.0f,  30.0f, "Bank angle during failsafe spiral (deg)");
CONFIG_FLOAT(CONFIG_KEY_FS_PITCH_DEG,  -3.0f, -20.0f, 0.0f, "Pitch angle during failsafe (deg)");
CONFIG_FLOAT(CONFIG_KEY_FS_THROTTLE,   0.0f,  0.0f,  1.0f,  "Throttle setting during failsafe");
CONFIG_UINT8(CONFIG_KEY_FS_MIN_LQ_ARM_PCT, 50,    20,    100,   "Min link quality % to arm");

// ═══════════════════════════════════════════════════════════════════════════
// CRSF Receiver Configuration
// ═══════════════════════════════════════════════════════════════════════════

CONFIG_FLOAT(CONFIG_KEY_CRSF_TRI_LOW,  0.33f, 0.1f, 0.4f, "TriState switch low threshold");
CONFIG_FLOAT(CONFIG_KEY_CRSF_TRI_HIGH, 0.66f, 0.6f, 0.9f, "TriState switch high threshold");

// ═══════════════════════════════════════════════════════════════════════════
// System Configuration
// ═══════════════════════════════════════════════════════════════════════════

CONFIG_STRING(CONFIG_KEY_SYS_AIRCRAFT_NAME, "ArduFlite", "Aircraft name for telemetry");

// ═══════════════════════════════════════════════════════════════════════════
// Web Server Configuration
// ═══════════════════════════════════════════════════════════════════════════

CONFIG_BOOL(CONFIG_KEY_WEB_ENABLED, false, "Enable WiFi AP and web server");
CONFIG_STRING(CONFIG_KEY_WEB_AP_SSID, "ArduFlite", "WiFi AP name (SSID)");
CONFIG_STRING(CONFIG_KEY_WEB_AP_PASS, "arduflite", "WiFi AP password (8+ characters; set before field use)");

// ═══════════════════════════════════════════════════════════════════════════
// MAVLink Configuration
// ═══════════════════════════════════════════════════════════════════════════

CONFIG_UINT8(CONFIG_KEY_MAV_SYSID,         1,     1,    250,     "MAVLink system ID (reboot to apply)");
CONFIG_BOOL(CONFIG_KEY_MAV_UART_ENABLED,   false, "MAVLink on the telemetry UART (reboot to apply)");
CONFIG_INT(CONFIG_KEY_MAV_UART_BAUD,       57600, 9600, 921600,  "Telemetry UART baud, matching the radio (reboot to apply)");
CONFIG_INT(CONFIG_KEY_MAV_UART_MAX_BPS,    4800,  1200, 1000000, "Telemetry UART send budget, bits/s (reboot to apply)");
CONFIG_BOOL(CONFIG_KEY_MAV_UART_WRITES,    false, "Accept parameter writes and commands over the telemetry UART (reboot to apply)");

#endif // CONFIG_SCHEMA_IMPL

#endif // CONFIG_SCHEMA_H
