/**
 * ArduFliteAttitudeController.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDU_FLITE_ATTITUDE_CONTROLLER_H
#define ARDU_FLITE_ATTITUDE_CONTROLLER_H

#include "src/orientation/FliteQuaternion.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/pid.h"
#include "include/ControllerTypes.h"

#include <Arduino.h>

/**
 * @brief ArduFliteAttitudeController class.
 *
 * This class implements the outer (attitude) control loop for the ArduFlite project.
 * It computes control outputs for roll, pitch, and yaw by comparing the current measured
 * orientation with a desired orientation. The computation involves decoupling yaw from roll
 * and pitch errors and using PID controllers to compute the necessary corrections.
 * A mutex is used to protect access to the desired orientation for thread safety.
 */
class ArduFliteAttitudeController
{
public:
    /**
     * @brief Default constructor.
     *
     * Creates mutex but does NOT initialize PIDs.
     * Call initFromConfig() after ConfigRegistry is ready.
     */
    ArduFliteAttitudeController();

    /**
     * @brief Initialize PID controllers from ConfigRegistry.
     *        Must be called after ConfigRegistry::init().
     */
    void initFromConfig();

    /**
     * @brief Sets the desired orientation using a quaternion.
     *
     * Updates the internal desired orientation in a thread-safe manner.
     *
     * @param qd The desired orientation as a quaternion.
     */
    void setAttitudeControlSetpointQuaternion(const FliteQuaternion &qd);

    /**
     * @brief Sets the desired orientation using Euler angles in radians.
     *
     * Converts the provided Euler angles (roll, pitch, yaw) to a quaternion and updates
     * the desired orientation.
     *
     * @param setpointRads Attitude setpoint in radians.
     */
    void setAttitudeControlSetpointRads(EulerAngles setpointRads);

    /**
     * @brief Sets the desired orientation using Euler angles in degrees.
     *
     * Converts the provided Euler angles (roll, pitch, yaw) from degrees to radians and
     * updates the desired orientation.
     *
     * @param setpointDegs  Attitude Setpoint in degrees.
     */
    void setAttitudeControlSetpoint(EulerAngles setpointDegs);

    /**
     * @brief Updates the attitude controller.
     *
     * Computes the error between the current measured orientation and the desired orientation.
     * The yaw error is computed separately and removed from both the desired and measured
     * quaternions. The remaining roll and pitch errors are converted to a rotation vector using
     * a logarithmic map. These errors are then fed into PID controllers to compute control outputs.
     *
     * @param measuredQ The measured orientation as a quaternion.
     * @param dt The time step in seconds.
     * @param rollOut Output control signal for roll (normalized to [-1, 1]).
     * @param pitchOut Output control signal for pitch (normalized to [-1, 1]).
     * @param yawOut Output control signal for yaw (normalized to [-1, 1]).
     */
    void update(const FliteQuaternion &measuredQ, float dt, EulerAngles &rateOut);

    /**
     * @brief Resets the PID controllers.
     *
     * Resets the integrators and derivative states for the roll, pitch, and yaw PID controllers.
     */
    void reset();

    /**
     * @brief Resets only the integral accumulators for all attitude PIDs.
     *
     * Called every outer-loop tick while in PREFLIGHT or LANDED state to prevent
     * I-term windup while the aircraft is idle on the ground before launch.
     */
    void resetIntegrals();

    // ─────────────────────────────────────────────────────────────────
    // Runtime Configuration Updates
    // ─────────────────────────────────────────────────────────────────

    /**
     * @brief Set the PID configuration for a specific axis.
     * @param loop The control loop type (ATTITUDE_ROLL_LOOP, ATTITUDE_PITCH_LOOP, ATTITUDE_YAW_LOOP)
     * @param config The PID configuration
     */
    void setPIDConfig(ControlLoopType loop, const PIDConfig& config);

    /**
     * @brief Set the attitude error deadband.
     * @param deadband Deadband in radians
     */
    void setDeadband(float deadband);

private:
    FliteQuaternion desiredQ;               //< The desired orientation.
    EulerAngles     attitudeSetpointDegs;   //< The desired orientation in degrees.
    float           deadbandRads;           //< Error deadband in radians.
    PID             pidRoll;                //< PID controller for roll.
    PID             pidPitch;               //< PID controller for pitch.
    PID             pidYaw;                 //< PID controller for yaw.

    /// Mutex to protect access to the desired orientation.
    SemaphoreHandle_t attitudeMutex;
};

#endif // ARDU_FLITE_ATTITUDE_CONTROLLER_H
