/**
 * ArduFliteRateController.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/controller/ArduFliteRateController.h"
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
void ArduFliteRateController::setDesiredRates(float rollRate, float pitchRate, float yawRate) 
{
    // Protect the update to desired rates.
    xSemaphoreTake(rateMutex, portMAX_DELAY);

    desiredRollRate  = rollRate;
    desiredPitchRate = pitchRate;
    desiredYawRate   = yawRate;

    xSemaphoreGive(rateMutex);
}

// Update the rate controller with the measured angular rates.
// The error is computed as (desiredRate - measuredRate) for each axis.
void ArduFliteRateController::update(float measuredRollRate, float measuredPitchRate, float measuredYawRate, float dt, float &rollOut, float &pitchOut, float &yawOut) 
{
    // Prevent a too-small dt.
    if (dt < 1e-3f) dt = 1e-3f;

    float localDesiredRoll, localDesiredPitch, localDesiredYaw;

    // Take mutex to safely read the desired rates.
    xSemaphoreTake(rateMutex, portMAX_DELAY);
    localDesiredRoll  = desiredRollRate;
    localDesiredPitch = desiredPitchRate;
    localDesiredYaw   = desiredYawRate;
    xSemaphoreGive(rateMutex);

    // Compute errors for each axis.
    float rollError  = desiredRollRate  - measuredRollRate;
    float pitchError = desiredPitchRate - measuredPitchRate;
    float yawError   = desiredYawRate   - measuredYawRate;

    // Compute raw PID outputs.
    float newRollOut  = pidRoll.update(rollError, dt);
    float newPitchOut = pidPitch.update(pitchError, dt);
    float newYawOut   = pidYaw.update(yawError, dt);

    // Apply an exponential moving average filter to smooth the output.
    filteredRollOutput  = outputAlpha * newRollOut  + (1.0f - outputAlpha) * filteredRollOutput;
    filteredPitchOutput = outputAlpha * newPitchOut + (1.0f - outputAlpha) * filteredPitchOutput;
    filteredYawOutput   = outputAlpha * newYawOut   + (1.0f - outputAlpha) * filteredYawOutput;

    // Use the filtered outputs as the final command.
    rollOut  = filteredRollOutput;
    pitchOut = filteredPitchOutput;
    yawOut   = filteredYawOutput;
}

// Reset all PID controllers.
void ArduFliteRateController::reset() 
{
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
}
