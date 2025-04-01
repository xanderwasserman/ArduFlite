#include "src/controller/ArduFliteRateController.h"

// Constructor with initial PID parameters.
// The gains here are sample values—you will need to tune them for your aircraft.
// The output limits are set to -1.0 and +1.0 so that the final servo commands remain normalized.
ArduFliteRateController::ArduFliteRateController()
    : desiredRollRate(0.0f), desiredPitchRate(0.0f), desiredYawRate(0.0f),
      pidRoll(  0.1f,   0.01f,  0.005f,     -1.0f,  1.0f),
      pidPitch( 0.1f,   0.01f,  0.005f,     -1.0f,  1.0f),
      pidYaw(   0.1f,   0.01f,  0.005f,     -1.0f,  1.0f)
{
    // Create the mutex to protect desired rate updates.
    rateMutex = xSemaphoreCreateMutex();
    if (rateMutex == NULL) {
        // Handle error accordingly (e.g., print an error message)
    }
}

// Set the desired angular rates (e.g., from the outer loop's output)
void ArduFliteRateController::setDesiredRates(float rollRate, float pitchRate, float yawRate) {
    // Protect the update to desired rates.
    xSemaphoreTake(rateMutex, portMAX_DELAY);

    desiredRollRate  = rollRate;
    desiredPitchRate = pitchRate;
    desiredYawRate   = yawRate;

    xSemaphoreGive(rateMutex);
}

// Update the rate controller with the measured angular rates.
// The error is computed as (desiredRate - measuredRate) for each axis.
void ArduFliteRateController::update(float measuredRollRate, float measuredPitchRate, float measuredYawRate, float dt, float &rollOut, float &pitchOut, float &yawOut) {
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

    // Update the PID controllers and compute final output commands.
    rollOut  = pidRoll.update(rollError, dt);
    pitchOut = pidPitch.update(pitchError, dt);
    yawOut   = pidYaw.update(yawError, dt);
}

// Reset all PID controllers.
void ArduFliteRateController::reset() {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
}
