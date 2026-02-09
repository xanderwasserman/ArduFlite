/**
 * ConfigKeys.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Compile-time key definitions for the persistent configuration system.
 *        Using #defines enables IDE autocomplete and compile-time typo detection.
 *        Keys use hierarchical dot notation: category.subcategory.param
 */
#ifndef CONFIG_KEYS_H
#define CONFIG_KEYS_H

// ═══════════════════════════════════════════════════════════════════════════
// Rate Controller PID (Inner Loop)
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_RATE_ROLL_KP         "rate.roll.kp"
#define CONFIG_KEY_RATE_ROLL_TI         "rate.roll.ti"
#define CONFIG_KEY_RATE_ROLL_TD         "rate.roll.td"
#define CONFIG_KEY_RATE_ROLL_OUTLIMIT   "rate.roll.outlimit"
#define CONFIG_KEY_RATE_ROLL_HEADROOM   "rate.roll.headroom"
#define CONFIG_KEY_RATE_ROLL_ALPHA      "rate.roll.alpha"
#define CONFIG_KEY_RATE_ROLL_ALL        "rate.roll.*"
#define CONFIG_KEY_RATE_ROLL_PREFIX     "rate.roll"

#define CONFIG_KEY_RATE_PITCH_KP        "rate.pitch.kp"
#define CONFIG_KEY_RATE_PITCH_TI        "rate.pitch.ti"
#define CONFIG_KEY_RATE_PITCH_TD        "rate.pitch.td"
#define CONFIG_KEY_RATE_PITCH_OUTLIMIT  "rate.pitch.outlimit"
#define CONFIG_KEY_RATE_PITCH_HEADROOM  "rate.pitch.headroom"
#define CONFIG_KEY_RATE_PITCH_ALPHA     "rate.pitch.alpha"
#define CONFIG_KEY_RATE_PITCH_ALL       "rate.pitch.*"
#define CONFIG_KEY_RATE_PITCH_PREFIX    "rate.pitch"

#define CONFIG_KEY_RATE_YAW_KP          "rate.yaw.kp"
#define CONFIG_KEY_RATE_YAW_TI          "rate.yaw.ti"
#define CONFIG_KEY_RATE_YAW_TD          "rate.yaw.td"
#define CONFIG_KEY_RATE_YAW_OUTLIMIT    "rate.yaw.outlimit"
#define CONFIG_KEY_RATE_YAW_HEADROOM    "rate.yaw.headroom"
#define CONFIG_KEY_RATE_YAW_ALPHA       "rate.yaw.alpha"
#define CONFIG_KEY_RATE_YAW_ALL         "rate.yaw.*"
#define CONFIG_KEY_RATE_YAW_PREFIX      "rate.yaw"

#define CONFIG_KEY_RATE_OUT_LP_ALPHA    "rate.out_lp_alpha"
#define CONFIG_KEY_RATE_ALL             "rate.*"

// ═══════════════════════════════════════════════════════════════════════════
// Attitude Controller PID (Outer Loop)
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_ATT_ROLL_KP          "att.roll.kp"
#define CONFIG_KEY_ATT_ROLL_TI          "att.roll.ti"
#define CONFIG_KEY_ATT_ROLL_TD          "att.roll.td"
#define CONFIG_KEY_ATT_ROLL_OUTLIMIT    "att.roll.outlimit"
#define CONFIG_KEY_ATT_ROLL_HEADROOM    "att.roll.headroom"
#define CONFIG_KEY_ATT_ROLL_ALPHA       "att.roll.alpha"
#define CONFIG_KEY_ATT_ROLL_ALL         "att.roll.*"
#define CONFIG_KEY_ATT_ROLL_PREFIX      "att.roll"

#define CONFIG_KEY_ATT_PITCH_KP         "att.pitch.kp"
#define CONFIG_KEY_ATT_PITCH_TI         "att.pitch.ti"
#define CONFIG_KEY_ATT_PITCH_TD         "att.pitch.td"
#define CONFIG_KEY_ATT_PITCH_OUTLIMIT   "att.pitch.outlimit"
#define CONFIG_KEY_ATT_PITCH_HEADROOM   "att.pitch.headroom"
#define CONFIG_KEY_ATT_PITCH_ALPHA      "att.pitch.alpha"
#define CONFIG_KEY_ATT_PITCH_ALL        "att.pitch.*"
#define CONFIG_KEY_ATT_PITCH_PREFIX     "att.pitch"

#define CONFIG_KEY_ATT_YAW_KP           "att.yaw.kp"
#define CONFIG_KEY_ATT_YAW_TI           "att.yaw.ti"
#define CONFIG_KEY_ATT_YAW_TD           "att.yaw.td"
#define CONFIG_KEY_ATT_YAW_OUTLIMIT     "att.yaw.outlimit"
#define CONFIG_KEY_ATT_YAW_HEADROOM     "att.yaw.headroom"
#define CONFIG_KEY_ATT_YAW_ALPHA        "att.yaw.alpha"
#define CONFIG_KEY_ATT_YAW_ALL          "att.yaw.*"
#define CONFIG_KEY_ATT_YAW_PREFIX       "att.yaw"

#define CONFIG_KEY_ATT_DEADBAND         "att.deadband"
#define CONFIG_KEY_ATT_ALL              "att.*"

// ═══════════════════════════════════════════════════════════════════════════
// Mixer Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_MIX_MAX_ATT_ROLL     "mix.max_att_roll"
#define CONFIG_KEY_MIX_MAX_ATT_PITCH    "mix.max_att_pitch"
#define CONFIG_KEY_MIX_MAX_ATT_YAW      "mix.max_att_yaw"
#define CONFIG_KEY_MIX_MAX_RATE_ROLL    "mix.max_rate_roll"
#define CONFIG_KEY_MIX_MAX_RATE_PITCH   "mix.max_rate_pitch"
#define CONFIG_KEY_MIX_MAX_RATE_YAW     "mix.max_rate_yaw"
#define CONFIG_KEY_MIX_ROLL_FROM_YAW    "mix.roll_from_yaw"
#define CONFIG_KEY_MIX_PITCH_FROM_ROLL  "mix.pitch_from_roll"
#define CONFIG_KEY_MIX_YAW_FROM_ROLL    "mix.yaw_from_roll"
#define CONFIG_KEY_MIX_ALL              "mix.*"

// ═══════════════════════════════════════════════════════════════════════════
// Servo Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_SERVO_WING_DESIGN    "servo.wing_design"    // 0=CONVENTIONAL, 1=DELTA_WING, 2=V_TAIL
#define CONFIG_KEY_SERVO_DUAL_AILERONS  "servo.dual_ailerons"
#define CONFIG_KEY_SERVO_MAX_DEG_SEC    "servo.max_deg_sec"
#define CONFIG_KEY_SERVO_MAX_THR_SEC    "servo.max_thr_sec"

#define CONFIG_KEY_SERVO_PITCH_MIN      "servo.pitch.min_pulse"
#define CONFIG_KEY_SERVO_PITCH_MAX      "servo.pitch.max_pulse"
#define CONFIG_KEY_SERVO_PITCH_NEUTRAL  "servo.pitch.neutral"
#define CONFIG_KEY_SERVO_PITCH_DEFL     "servo.pitch.deflection"
#define CONFIG_KEY_SERVO_PITCH_INV      "servo.pitch.invert"
#define CONFIG_KEY_SERVO_PITCH_ALL      "servo.pitch.*"

#define CONFIG_KEY_SERVO_YAW_MIN        "servo.yaw.min_pulse"
#define CONFIG_KEY_SERVO_YAW_MAX        "servo.yaw.max_pulse"
#define CONFIG_KEY_SERVO_YAW_NEUTRAL    "servo.yaw.neutral"
#define CONFIG_KEY_SERVO_YAW_DEFL       "servo.yaw.deflection"
#define CONFIG_KEY_SERVO_YAW_INV        "servo.yaw.invert"
#define CONFIG_KEY_SERVO_YAW_ALL        "servo.yaw.*"

#define CONFIG_KEY_SERVO_LAIL_MIN       "servo.lail.min_pulse"
#define CONFIG_KEY_SERVO_LAIL_MAX       "servo.lail.max_pulse"
#define CONFIG_KEY_SERVO_LAIL_NEUTRAL   "servo.lail.neutral"
#define CONFIG_KEY_SERVO_LAIL_DEFL      "servo.lail.deflection"
#define CONFIG_KEY_SERVO_LAIL_INV       "servo.lail.invert"
#define CONFIG_KEY_SERVO_LAIL_ALL       "servo.lail.*"

#define CONFIG_KEY_SERVO_RAIL_MIN       "servo.rail.min_pulse"
#define CONFIG_KEY_SERVO_RAIL_MAX       "servo.rail.max_pulse"
#define CONFIG_KEY_SERVO_RAIL_NEUTRAL   "servo.rail.neutral"
#define CONFIG_KEY_SERVO_RAIL_DEFL      "servo.rail.deflection"
#define CONFIG_KEY_SERVO_RAIL_INV       "servo.rail.invert"
#define CONFIG_KEY_SERVO_RAIL_ALL       "servo.rail.*"

#define CONFIG_KEY_SERVO_THR_MIN        "servo.thr.min_pulse"
#define CONFIG_KEY_SERVO_THR_MAX        "servo.thr.max_pulse"
#define CONFIG_KEY_SERVO_THR_ALL        "servo.thr.*"

#define CONFIG_KEY_SERVO_ALL            "servo.*"

// ═══════════════════════════════════════════════════════════════════════════
// IMU Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_IMU_ACCEL_ALPHA      "imu.accel_alpha"
#define CONFIG_KEY_IMU_GYRO_ALPHA       "imu.gyro_alpha"
#define CONFIG_KEY_IMU_MAG_ALPHA        "imu.mag_alpha"
#define CONFIG_KEY_IMU_ALTI_ALPHA       "imu.alti_alpha"
#define CONFIG_KEY_IMU_MADGWICK_BETA    "imu.madgwick_beta"
#define CONFIG_KEY_IMU_MAX_ACCEL_G      "imu.max_accel_g"
#define CONFIG_KEY_IMU_MAX_GYRO_DPS     "imu.max_gyro_dps"
#define CONFIG_KEY_IMU_FAIL_THRESHOLD   "imu.fail_threshold"
#define CONFIG_KEY_IMU_GYRO_BIAS_MAX    "imu.gyro_bias_max"
#define CONFIG_KEY_IMU_EXPECTED_G       "imu.expected_g"
#define CONFIG_KEY_IMU_GRAVITY_TOL      "imu.gravity_tol"
#define CONFIG_KEY_IMU_ALL              "imu.*"

// ═══════════════════════════════════════════════════════════════════════════
// Failsafe Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_FS_BANK_DEG          "failsafe.bank_deg"
#define CONFIG_KEY_FS_PITCH_DEG         "failsafe.pitch_deg"
#define CONFIG_KEY_FS_THROTTLE          "failsafe.throttle"
#define CONFIG_KEY_FS_MIN_LQ_ARM        "failsafe.min_lq_arm"
#define CONFIG_KEY_FS_ALL               "failsafe.*"

// ═══════════════════════════════════════════════════════════════════════════
// CRSF Receiver Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_CRSF_TRI_LOW         "crsf.tri_low"
#define CONFIG_KEY_CRSF_TRI_HIGH        "crsf.tri_high"
#define CONFIG_KEY_CRSF_ALL             "crsf.*"

// ═══════════════════════════════════════════════════════════════════════════
// System Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_SYS_AIRCRAFT_NAME    "sys.aircraft_name"
#define CONFIG_KEY_SYS_ALL              "sys.*"

// ═══════════════════════════════════════════════════════════════════════════
// Web Server Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_WEB_ENABLED          "web.enabled"
#define CONFIG_KEY_WEB_AP_SSID          "web.ap_ssid"
#define CONFIG_KEY_WEB_AP_PASS          "web.ap_pass"
#define CONFIG_KEY_WEB_ALL              "web.*"

// ═══════════════════════════════════════════════════════════════════════════
// All Configuration (root wildcard)
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_ALL                  "*"

// ═══════════════════════════════════════════════════════════════════════════
// Note: Pin configurations are compile-time constants in PinConfiguration.h
// Board selection is done via BOARD_TYPE macro at build time.
// ═══════════════════════════════════════════════════════════════════════════

#endif // CONFIG_KEYS_H
