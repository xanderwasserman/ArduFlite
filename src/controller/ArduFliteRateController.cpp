/**
 * ArduFliteRateController.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/controller/ArduFliteRateController.h"
#include "src/orientation/ArduFliteIMU.h"
#include "include/ArduFlite.h"
#include "src/utils/Logging.h"

// Constructor with initial PID parameters.
// The output limits are set to -1.0 and +1.0 so that the final servo commands remain normalized.
ArduFliteRateController::ArduFliteRateController()
    : pidRoll(RateControllerConfig::DEFAULT_ROLL_PID),
      pidPitch(RateControllerConfig::DEFAULT_PITCH_PID),
      pidYaw(RateControllerConfig::DEFAULT_YAW_PID),
      outputAlpha(RateControllerConfig::outLpAlpha)
{
    // Create the mutex to protect desired rate updates.
    rateMutex = xSemaphoreCreateMutex();
    if (rateMutex == NULL) {
        // Handle error accordingly (e.g., print an error message)
        LOG_ERR("Failed to create ArduFliteRateController mutex!");
    }
}

// Set the desired angular rates (e.g., from the outer loop's output)
void ArduFliteRateController::setRateControlSetpoint(EulerAngles setpoint) 
{
    // Protect the update to desired rates.
    {
        SemaphoreLock lock(rateMutex);
        setpointRate = setpoint;
    }
}

// Update the rate controller with the measured angular rates.
// The error is computed as (desiredRate - measuredRate) for each axis.
void ArduFliteRateController::update(Vector3 measuredRate, float dt, EulerAngles &actuatorOut) 
{
    // Prevent a too-small dt.
    if (dt < 1e-3f) dt = 1e-3f;

    float localDesiredRoll, localDesiredPitch, localDesiredYaw;
    EulerAngles localSetpointRate;

    // Take mutex to safely read the desired rates.
    {
        SemaphoreLock lock(rateMutex);
        localSetpointRate = setpointRate;
    }

    // Compute errors for each axis.
    float rollError  = localSetpointRate.roll  - measuredRate.x;
    float pitchError = localSetpointRate.pitch - measuredRate.y;
    float yawError   = localSetpointRate.yaw   - measuredRate.z;

    // Compute raw PID outputs.
    float newRollOut  = pidRoll.update(rollError, dt);
    float newPitchOut = pidPitch.update(pitchError, dt);
    float newYawOut   = pidYaw.update(yawError, dt);

    // Apply an exponential moving average filter to smooth the output.
    filteredRateOutput.roll  = outputAlpha * newRollOut  + (1.0f - outputAlpha) * filteredRateOutput.roll;
    filteredRateOutput.pitch = outputAlpha * newPitchOut + (1.0f - outputAlpha) * filteredRateOutput.pitch;
    filteredRateOutput.yaw   = outputAlpha * newYawOut   + (1.0f - outputAlpha) * filteredRateOutput.yaw;

    // Final safety clamp: ensure all outputs are within valid servo command range [-1, 1]
    // This protects against any corruption or drift in the EMA filter state.
    actuatorOut.roll  = constrain(filteredRateOutput.roll,  -1.0f, 1.0f);
    actuatorOut.pitch = constrain(filteredRateOutput.pitch, -1.0f, 1.0f);
    actuatorOut.yaw   = constrain(filteredRateOutput.yaw,   -1.0f, 1.0f);
}

// Reset all PID controllers.
void ArduFliteRateController::reset() 
{
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
}
