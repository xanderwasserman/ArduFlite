/**
 * ArduFliteRateController.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDU_FLITE_RATE_CONTROLLER_H
#define ARDU_FLITE_RATE_CONTROLLER_H

#include <Arduino.h>
#include "src/controller/pid.h"
#include "include/ControllerConfiguration.h"
#include "src/orientation/ArduFliteIMU.h"

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
    void setRateControlSetpoint(EulerAngles setpoint);

    // Main update function:
    // measuredRollRate, measuredPitchRate, measuredYawRate: measured angular rates from the IMU.
    // dt: time step in seconds.
    // rollOut, pitchOut, yawOut: final control signals (normalized to [-1, 1]) to drive the servos.
    void update(Vector3 measuredRate, float dt, EulerAngles &actuatorOut);

    // Reset the PID controllers' integrators.
    void reset();

private:
    // Desired angular rates (set by the outer loop)
    EulerAngles setpointRate        {0.0f};
    EulerAngles filteredRateOutput  {0.0f};
    float outputAlpha               = 0.1f;

    // PID controllers for each axis.
    PID pidRoll, pidPitch, pidYaw;

    // Mutex for protecting access to class state.
    SemaphoreHandle_t rateMutex;
};

#endif // ARDU_FLITE_RATE_CONTROLLER_H
