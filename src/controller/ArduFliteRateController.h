#ifndef ARDU_FLITE_RATE_CONTROLLER_H
#define ARDU_FLITE_RATE_CONTROLLER_H

#include <Arduino.h>
#include "src/controller/pid.h"
#include "src/controller/ControllerConfig.h"

/**
 * @brief The ArduFliteRateController class implements an inner loop
 * rate controller for the aircraft. It uses three PID controllers (one per axis)
 * to compute final servo commands based on the error between the desired and
 * measured angular rates.
 *
 * All angular rates should be in the same units (e.g., degrees per second).
 * The final servo outputs are normalized to the range [-1, 1].
 */
class ArduFliteRateController 
{
public:
    // Constructor: Initializes the PID controllers with appropriate gains and output limits.
    ArduFliteRateController();

    // Set the desired angular rates (roll, pitch, yaw). Units can be degrees per second.
    void setDesiredRates(float rollRate, float pitchRate, float yawRate);

    // Main update function:
    // measuredRollRate, measuredPitchRate, measuredYawRate: measured angular rates from the IMU.
    // dt: time step in seconds.
    // rollOut, pitchOut, yawOut: final control signals (normalized to [-1, 1]) to drive the servos.
    void update(float measuredRollRate, float measuredPitchRate, float measuredYawRate, float dt,
                float &rollOut, float &pitchOut, float &yawOut);

    // Reset the PID controllers' integrators.
    void reset();

private:
    // Desired angular rates (set by the outer loop)
    float desiredRollRate = 0.0f;
    float desiredPitchRate = 0.0f;
    float desiredYawRate = 0.0f;

    float filteredRollOutput = 0.0f;
    float filteredPitchOutput = 0.0f;
    float filteredYawOutput = 0.0f;
    float outputAlpha = 0.01f;

    // PID controllers for each axis.
    PID pidRoll, pidPitch, pidYaw;

    // Mutex for protecting access to class state.
    SemaphoreHandle_t rateMutex;
};

#endif // ARDU_FLITE_RATE_CONTROLLER_H
