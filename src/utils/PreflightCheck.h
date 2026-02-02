/**
 * PreflightCheck.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 01 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 * 
 * @file PreflightCheck.h
 * @brief Preflight validation system for ArduFlite.
 * 
 * Validates all critical systems before allowing arming:
 * - IMU health and stability
 * - Receiver link quality
 * - Throttle position (must be at minimum)
 * - Gyro bias within acceptable limits
 * - Accelerometer gravity reading
 */

#ifndef ARDUFLITE_PREFLIGHT_CHECK_H
#define ARDUFLITE_PREFLIGHT_CHECK_H

#include <Arduino.h>

// Forward declarations
class ArduFliteIMU;
class ArduFliteController;
class ArdufliteCRSFReceiver;

/**
 * @brief Result of individual preflight checks.
 */
struct PreflightResult
{
    bool imuHealthy         = false;    ///< IMU returning valid data
    bool gyroStable         = false;    ///< Gyro bias within limits
    bool accelValid         = false;    ///< Accelerometer reading ~1g
    bool receiverLinked     = false;    ///< Receiver link quality OK
    bool throttleMinimum    = false;    ///< Throttle at minimum position
    
    /**
     * @brief Returns true only if ALL checks passed.
     */
    bool allPassed() const
    {
        return imuHealthy && gyroStable && accelValid && 
               receiverLinked && throttleMinimum;
    }
};

/**
 * @brief Preflight check utility for ArduFlite.
 * 
 * Standalone module that validates all critical systems before arming.
 * Uses dependency injection to access system components.
 */
namespace PreflightCheck
{
    /**
     * @brief Run all preflight checks.
     * 
     * Validates IMU health, gyro stability, accelerometer gravity reading,
     * receiver link quality, and throttle position.
     * 
     * @param imu Pointer to the ArduFliteIMU instance
     * @param controller Pointer to the ArduFliteController instance
     * @param receiver Pointer to the ArdufliteCRSFReceiver instance (may be nullptr for PWM receivers)
     * @return PreflightResult with individual check results
     */
    PreflightResult runAllChecks(
        ArduFliteIMU* imu,
        ArduFliteController* controller,
        ArdufliteCRSFReceiver* receiver
    );

    /**
     * @brief Check if IMU is healthy and returning valid data.
     * 
     * @param imu Pointer to the ArduFliteIMU instance
     * @return true if IMU is healthy
     */
    bool checkIMUHealth(ArduFliteIMU* imu);

    /**
     * @brief Check if gyro bias is within acceptable limits.
     * 
     * Reads current gyro values and verifies they are below GYRO_BIAS_MAX_DPS.
     * Aircraft should be stationary during this check.
     * 
     * @param imu Pointer to the ArduFliteIMU instance
     * @return true if gyro bias is acceptable
     */
    bool checkGyroStability(ArduFliteIMU* imu);

    /**
     * @brief Check if accelerometer is reading approximately 1g.
     * 
     * When stationary and level, the accelerometer should read close to 1g.
     * This validates the sensor is working correctly.
     * 
     * @param imu Pointer to the ArduFliteIMU instance
     * @return true if accelerometer reading is within expected range
     */
    bool checkAccelerometer(ArduFliteIMU* imu);

    /**
     * @brief Check if receiver link quality is sufficient for flight.
     * 
     * Requires link quality >= MIN_LINK_QUALITY_ARM from configuration.
     * 
     * @param receiver Pointer to the ArdufliteCRSFReceiver instance
     * @return true if link quality is acceptable (or no CRSF receiver configured)
     */
    bool checkReceiverLink(ArdufliteCRSFReceiver* receiver);

    /**
     * @brief Check if throttle is at minimum position.
     * 
     * Prevents arming with throttle raised to avoid unexpected motor spin-up.
     * 
     * @param controller Pointer to the ArduFliteController instance
     * @return true if throttle is at minimum (< 10%)
     */
    bool checkThrottleMinimum(ArduFliteController* controller);

    /**
     * @brief Log detailed preflight check results.
     * 
     * @param result The PreflightResult to log
     */
    void logResults(const PreflightResult& result);

} // namespace PreflightCheck

#endif // ARDUFLITE_PREFLIGHT_CHECK_H
