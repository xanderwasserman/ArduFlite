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
#define CONFIG_KEY_RATE_ROLL_TI_S         "rate.roll.ti_s"
#define CONFIG_KEY_RATE_ROLL_TD_S         "rate.roll.td_s"
#define CONFIG_KEY_RATE_ROLL_OUTLIMIT   "rate.roll.outlimit"
#define CONFIG_KEY_RATE_ROLL_HEADROOM   "rate.roll.headroom"
#define CONFIG_KEY_RATE_ROLL_ALPHA      "rate.roll.alpha"
#define CONFIG_KEY_RATE_ROLL_ALL        "rate.roll.*"
#define CONFIG_KEY_RATE_ROLL_PREFIX     "rate.roll"

#define CONFIG_KEY_RATE_PITCH_KP        "rate.pitch.kp"
#define CONFIG_KEY_RATE_PITCH_TI_S        "rate.pitch.ti_s"
#define CONFIG_KEY_RATE_PITCH_TD_S        "rate.pitch.td_s"
#define CONFIG_KEY_RATE_PITCH_OUTLIMIT  "rate.pitch.outlimit"
#define CONFIG_KEY_RATE_PITCH_HEADROOM  "rate.pitch.headroom"
#define CONFIG_KEY_RATE_PITCH_ALPHA     "rate.pitch.alpha"
#define CONFIG_KEY_RATE_PITCH_ALL       "rate.pitch.*"
#define CONFIG_KEY_RATE_PITCH_PREFIX    "rate.pitch"

#define CONFIG_KEY_RATE_YAW_KP          "rate.yaw.kp"
#define CONFIG_KEY_RATE_YAW_TI_S          "rate.yaw.ti_s"
#define CONFIG_KEY_RATE_YAW_TD_S          "rate.yaw.td_s"
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
#define CONFIG_KEY_ATT_ROLL_TI_S          "att.roll.ti_s"
#define CONFIG_KEY_ATT_ROLL_TD_S          "att.roll.td_s"
#define CONFIG_KEY_ATT_ROLL_OUTLIMIT_DPS    "att.roll.outlimit_dps"
#define CONFIG_KEY_ATT_ROLL_HEADROOM    "att.roll.headroom"
#define CONFIG_KEY_ATT_ROLL_ALPHA       "att.roll.alpha"
#define CONFIG_KEY_ATT_ROLL_ALL         "att.roll.*"
#define CONFIG_KEY_ATT_ROLL_PREFIX      "att.roll"

#define CONFIG_KEY_ATT_PITCH_KP         "att.pitch.kp"
#define CONFIG_KEY_ATT_PITCH_TI_S         "att.pitch.ti_s"
#define CONFIG_KEY_ATT_PITCH_TD_S         "att.pitch.td_s"
#define CONFIG_KEY_ATT_PITCH_OUTLIMIT_DPS   "att.pitch.outlimit_dps"
#define CONFIG_KEY_ATT_PITCH_HEADROOM   "att.pitch.headroom"
#define CONFIG_KEY_ATT_PITCH_ALPHA      "att.pitch.alpha"
#define CONFIG_KEY_ATT_PITCH_ALL        "att.pitch.*"
#define CONFIG_KEY_ATT_PITCH_PREFIX     "att.pitch"

#define CONFIG_KEY_ATT_YAW_KP           "att.yaw.kp"
#define CONFIG_KEY_ATT_YAW_TI_S           "att.yaw.ti_s"
#define CONFIG_KEY_ATT_YAW_TD_S           "att.yaw.td_s"
#define CONFIG_KEY_ATT_YAW_OUTLIMIT_DPS     "att.yaw.outlimit_dps"
#define CONFIG_KEY_ATT_YAW_HEADROOM     "att.yaw.headroom"
#define CONFIG_KEY_ATT_YAW_ALPHA        "att.yaw.alpha"
#define CONFIG_KEY_ATT_YAW_ALL          "att.yaw.*"
#define CONFIG_KEY_ATT_YAW_PREFIX       "att.yaw"

#define CONFIG_KEY_ATT_DEADBAND_RAD         "att.deadband_rad"
#define CONFIG_KEY_ATT_ALL              "att.*"

// ═══════════════════════════════════════════════════════════════════════════
// Mixer Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_MIX_MAX_ATT_ROLL_DEG     "mix.max_att_roll_deg"
#define CONFIG_KEY_MIX_MAX_ATT_PITCH_DEG    "mix.max_att_pitch_deg"
#define CONFIG_KEY_MIX_MAX_ATT_YAW_DEG      "mix.max_att_yaw_deg"
#define CONFIG_KEY_MIX_MAX_RATE_ROLL_DPS    "mix.max_rate_roll_dps"
#define CONFIG_KEY_MIX_MAX_RATE_PITCH_DPS   "mix.max_rate_pitch_dps"
#define CONFIG_KEY_MIX_MAX_RATE_YAW_DPS     "mix.max_rate_yaw_dps"
#define CONFIG_KEY_MIX_ROLL_FROM_YAW    "mix.roll_from_yaw"
#define CONFIG_KEY_MIX_PITCH_FROM_ROLL  "mix.pitch_from_roll"
#define CONFIG_KEY_MIX_YAW_FROM_ROLL    "mix.yaw_from_roll"
#define CONFIG_KEY_MIX_ALL              "mix.*"

// ═══════════════════════════════════════════════════════════════════════════
// Servo Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_SERVO_WING_DESIGN    "servo.wing_design"    // 0=CONVENTIONAL, 1=DELTA_WING, 2=V_TAIL
#define CONFIG_KEY_SERVO_DUAL_AILERONS  "servo.dual_ailerons"
#define CONFIG_KEY_SERVO_MAX_SLEW_DPS    "servo.max_slew_dps"
#define CONFIG_KEY_SERVO_MAX_THR_SLEW_PER_S    "servo.max_thr_slew_per_s"

#define CONFIG_KEY_SERVO_PITCH_MIN_US      "servo.pitch.min_pulse_us"
#define CONFIG_KEY_SERVO_PITCH_MAX_US      "servo.pitch.max_pulse_us"
#define CONFIG_KEY_SERVO_PITCH_NEUTRAL_DEG  "servo.pitch.neutral_deg"
#define CONFIG_KEY_SERVO_PITCH_DEFL_DEG     "servo.pitch.deflection_deg"
#define CONFIG_KEY_SERVO_PITCH_INV      "servo.pitch.invert"
#define CONFIG_KEY_SERVO_PITCH_ALL      "servo.pitch.*"

#define CONFIG_KEY_SERVO_YAW_MIN_US        "servo.yaw.min_pulse_us"
#define CONFIG_KEY_SERVO_YAW_MAX_US        "servo.yaw.max_pulse_us"
#define CONFIG_KEY_SERVO_YAW_NEUTRAL_DEG    "servo.yaw.neutral_deg"
#define CONFIG_KEY_SERVO_YAW_DEFL_DEG       "servo.yaw.deflection_deg"
#define CONFIG_KEY_SERVO_YAW_INV        "servo.yaw.invert"
#define CONFIG_KEY_SERVO_YAW_ALL        "servo.yaw.*"

#define CONFIG_KEY_SERVO_LAIL_MIN_US       "servo.lail.min_pulse_us"
#define CONFIG_KEY_SERVO_LAIL_MAX_US       "servo.lail.max_pulse_us"
#define CONFIG_KEY_SERVO_LAIL_NEUTRAL_DEG   "servo.lail.neutral_deg"
#define CONFIG_KEY_SERVO_LAIL_DEFL_DEG      "servo.lail.deflection_deg"
#define CONFIG_KEY_SERVO_LAIL_INV       "servo.lail.invert"
#define CONFIG_KEY_SERVO_LAIL_ALL       "servo.lail.*"

#define CONFIG_KEY_SERVO_RAIL_MIN_US       "servo.rail.min_pulse_us"
#define CONFIG_KEY_SERVO_RAIL_MAX_US       "servo.rail.max_pulse_us"
#define CONFIG_KEY_SERVO_RAIL_NEUTRAL_DEG   "servo.rail.neutral_deg"
#define CONFIG_KEY_SERVO_RAIL_DEFL_DEG      "servo.rail.deflection_deg"
#define CONFIG_KEY_SERVO_RAIL_INV       "servo.rail.invert"
#define CONFIG_KEY_SERVO_RAIL_ALL       "servo.rail.*"

#define CONFIG_KEY_SERVO_THR_MIN_US        "servo.thr.min_pulse_us"
#define CONFIG_KEY_SERVO_THR_MAX_US        "servo.thr.max_pulse_us"
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
#define CONFIG_KEY_IMU_GYRO_BIAS_MAX_DPS    "imu.gyro_bias_max_dps"
#define CONFIG_KEY_IMU_EXPECTED_G       "imu.expected_g"
#define CONFIG_KEY_IMU_GRAVITY_TOL_G      "imu.gravity_tol_g"
#define CONFIG_KEY_IMU_ALL              "imu.*"

// ═══════════════════════════════════════════════════════════════════════════
// Failsafe Configuration
// ═══════════════════════════════════════════════════════════════════════════
#define CONFIG_KEY_FS_BANK_DEG          "failsafe.bank_deg"
#define CONFIG_KEY_FS_PITCH_DEG         "failsafe.pitch_deg"
#define CONFIG_KEY_FS_THROTTLE          "failsafe.throttle"
#define CONFIG_KEY_FS_MIN_LQ_ARM_PCT        "failsafe.min_lq_arm_pct"
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
