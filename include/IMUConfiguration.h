/**
 * IMUConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 24 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include "src/orientation/ArduFliteIMU.h"

// Set the default Barometer type. Change this definition to BARO_TYPE_BMP280 to use the BMP280.
#ifndef BARO_TYPE
    #define BARO_TYPE BARO_TYPE_BMP280
#endif


// Set the default IMU type. Change this definition to IMU_TYPE_MPU9250 to use the 9250.
// #define IMU_TYPE IMU_TYPE_MPU6500
#ifndef IMU_TYPE
    #define IMU_TYPE IMU_TYPE_MPU6500
#endif

namespace IMUConfig 
{
    // Low-pass filter alpha values (higher = less smoothing)
    constexpr float ACCEL_ALPHA     = 0.05f;
    constexpr float GYRO_ALPHA      = 0.05f;
    constexpr float MAG_ALPHA       = 0.1f;
    constexpr float ALTI_ALPHA      = 0.005f;

    // ─────────────────────────────────────────────────────────────────
    // IMU Health Monitoring Configuration
    // ─────────────────────────────────────────────────────────────────
    
    /// Maximum valid accelerometer magnitude (g). Values beyond this are considered invalid.
    constexpr float MAX_ACCEL_G                     = 16.0f;
    
    /// Maximum valid gyroscope rate (deg/s). Values beyond this are considered invalid.
    constexpr float MAX_GYRO_DPS                    = 2000.0f;
    
    /// Number of consecutive invalid readings before IMU is marked unhealthy.
    constexpr uint8_t CONSECUTIVE_FAILURES_THRESHOLD = 5;

    // ─────────────────────────────────────────────────────────────────
    // Preflight Check Configuration
    // ─────────────────────────────────────────────────────────────────
    
    /// Maximum acceptable gyro bias during preflight (deg/s). 
    /// If any axis exceeds this, preflight check fails.
    constexpr float GYRO_BIAS_MAX_DPS               = 5.0f;
    
    /// Expected gravity magnitude with tolerance (g). 
    /// Accelerometer should read close to 1g when stationary.
    constexpr float EXPECTED_GRAVITY_G              = 1.0f;
    constexpr float GRAVITY_TOLERANCE_G             = 0.15f;

} // namespace IMUConfig

#endif // IMU_CONFIG_H
