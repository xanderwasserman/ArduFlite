/**
 * PreflightCheck.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 01 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 * 
 * @file PreflightCheck.cpp
 * @brief Implementation of preflight validation system.
 */

#include "src/utils/PreflightCheck.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteController.h"
#include "src/receiver/crsf/ArdufliteCRSFReceiver.h"
#include "src/utils/Logging.h"
#include "include/IMUConfiguration.h"
#include "include/ReceiverConfiguration.h"

#include <math.h>

namespace PreflightCheck
{

PreflightResult runAllChecks(
    ArduFliteIMU* imu,
    ArduFliteController* controller,
    ArdufliteCRSFReceiver* receiver)
{
    PreflightResult result;
    
    LOG_INF("=== Running Preflight Checks ===");
    
    result.imuHealthy       = checkIMUHealth(imu);
    result.gyroStable       = checkGyroStability(imu);
    result.accelValid       = checkAccelerometer(imu);
    result.receiverLinked   = checkReceiverLink(receiver);
    result.throttleMinimum  = checkThrottleMinimum(controller);
    
    logResults(result);
    
    return result;
}

bool checkIMUHealth(ArduFliteIMU* imu)
{
    if (imu == nullptr)
    {
        LOG_ERR("Preflight: IMU pointer is null!");
        return false;
    }
    
    bool healthy = imu->isHealthy();
    if (!healthy)
    {
        LOG_ERR("Preflight: IMU is reporting unhealthy status");
    }
    
    return healthy;
}

bool checkGyroStability(ArduFliteIMU* imu)
{
    if (imu == nullptr)
    {
        return false;
    }
    
    // Average multiple gyro samples to filter out noise/vibration
    // Aircraft should be stationary during preflight check
    constexpr int NUM_SAMPLES = 10;
    constexpr int SAMPLE_DELAY_MS = 10;
    
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        Vector3 gyro = imu->getGyro();
        sumX += gyro.x;
        sumY += gyro.y;
        sumZ += gyro.z;
        
        if (i < NUM_SAMPLES - 1)
        {
            delay(SAMPLE_DELAY_MS);  // Brief delay between samples
        }
    }
    
    // Compute average
    float avgX = sumX / NUM_SAMPLES;
    float avgY = sumY / NUM_SAMPLES;
    float avgZ = sumZ / NUM_SAMPLES;
    
    // Check if average gyro values are within acceptable bias threshold
    bool stable = (fabsf(avgX) < IMUConfig::GYRO_BIAS_MAX_DPS) &&
                  (fabsf(avgY) < IMUConfig::GYRO_BIAS_MAX_DPS) &&
                  (fabsf(avgZ) < IMUConfig::GYRO_BIAS_MAX_DPS);
    
    if (!stable)
    {
        LOG_ERR("Preflight: Gyro bias too high! Avg X=%.2f Y=%.2f Z=%.2f (max=%.2f)",
                avgX, avgY, avgZ, IMUConfig::GYRO_BIAS_MAX_DPS);
    }
    else
    {
        LOG_INF("Preflight: Gyro bias OK (avg X=%.2f Y=%.2f Z=%.2f)", avgX, avgY, avgZ);
    }
    
    return stable;
}

bool checkAccelerometer(ArduFliteIMU* imu)
{
    if (imu == nullptr)
    {
        return false;
    }
    
    Vector3 accel = imu->getAcceleration();
    
    // Calculate total acceleration magnitude
    float magnitude = sqrtf(accel.x * accel.x + 
                            accel.y * accel.y + 
                            accel.z * accel.z);
    
    // Check if magnitude is close to 1g (with tolerance)
    float deviation = fabsf(magnitude - IMUConfig::EXPECTED_GRAVITY_G);
    bool valid = (deviation < IMUConfig::GRAVITY_TOLERANCE_G);
    
    if (!valid)
    {
        LOG_ERR("Preflight: Accelerometer reading invalid! Magnitude=%.3fg (expected=%.2f±%.2f)",
                magnitude, IMUConfig::EXPECTED_GRAVITY_G, IMUConfig::GRAVITY_TOLERANCE_G);
    }
    
    return valid;
}

bool checkReceiverLink(ArdufliteCRSFReceiver* receiver)
{
    // If no CRSF receiver is configured, pass this check
    // (might be using PWM receiver or testing without receiver)
    if (receiver == nullptr)
    {
        LOG_WARN("Preflight: No CRSF receiver configured, skipping link check");
        return true;
    }
    
    crsfLinkStatistics_t stats;
    bool hasStats = receiver->getLinkStats(stats);
    
    if (!hasStats)
    {
        LOG_ERR("Preflight: No receiver link statistics available");
        return false;
    }
    
    bool linkOk = (stats.uplink_Link_quality >= FailsafeConfig::MIN_LINK_QUALITY_ARM);
    
    if (!linkOk)
    {
        LOG_ERR("Preflight: Link quality too low! LQ=%u%% (min=%u%%)",
                stats.uplink_Link_quality, FailsafeConfig::MIN_LINK_QUALITY_ARM);
    }
    
    return linkOk;
}

bool checkThrottleMinimum(ArduFliteController* controller)
{
    if (controller == nullptr)
    {
        LOG_ERR("Preflight: Controller pointer is null!");
        return false;
    }
    
    // Require throttle cut to be enabled before arming.
    // This prevents accidental motor spin-up when arming.
    bool throttleCut = controller->isThrottleCut();
    
    if (!throttleCut)
    {
        LOG_ERR("Preflight: Throttle is not cut! Cannot arm with throttle enabled.");
        return false;
    }
    
    return true;
}

void logResults(const PreflightResult& result)
{
    LOG_INF("=== Preflight Results ===");
    LOG_INF("  IMU Health:     %s", result.imuHealthy ? "PASS" : "FAIL");
    LOG_INF("  Gyro Stability: %s", result.gyroStable ? "PASS" : "FAIL");
    LOG_INF("  Accelerometer:  %s", result.accelValid ? "PASS" : "FAIL");
    LOG_INF("  Receiver Link:  %s", result.receiverLinked ? "PASS" : "FAIL");
    LOG_INF("  Throttle Min:   %s", result.throttleMinimum ? "PASS" : "FAIL");
    LOG_INF("  Overall:        %s", result.allPassed() ? "PASS - Ready to arm" : "FAIL - Cannot arm");
}

} // namespace PreflightCheck
