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
    constexpr float ACCEL_ALPHA     = 0.05f;
    constexpr float GYRO_ALPHA      = 0.05f;
    constexpr float MAG_ALPHA       = 0.1f;
    constexpr float ALTI_ALPHA      = 0.005f;

} // namespace IMUConfig

#endif // IMU_CONFIG_H
