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

namespace IMUConfig 
{
    constexpr float ACCEL_ALPHA     = 0.05f;
    constexpr float GYRO_ALPHA      = 0.05f;
    constexpr float MAG_ALPHA       = 0.1f;
    constexpr float ALTI_ALPHA      = 0.005f;

} // namespace IMUConfig

#endif // IMU_CONFIG_H
