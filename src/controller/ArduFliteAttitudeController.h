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

#include <mutex>

#include "src/hal/platform/Mutex.h"

#include "src/orientation/FliteQuaternion.h"
#include "src/core/FlightTypes.h"
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
    /// Supply the lock guarding this controller's state. Must be called before
    /// any other method; every one of them becomes a no-op until it is.
    void setMutex(arduflite::hal::Mutex* mutex) { attitudeMutex = mutex; }

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
     * @brief Sets the desired orientation using Euler angles in degrees.
     *
     * Converts the provided Euler angles (roll, pitch, yaw) from degrees to radians and
     * updates the desired orientation.
     *
     * @param setpointDegs  Attitude Setpoint in degrees.
     */
    void setAttitudeControlSetpoint(AttitudeDeg setpointDegs);

    /**
     * @brief Updates the attitude controller.
     *
     * Computes the error between the current measured orientation and the desired orientation.
     * The yaw error is computed separately and removed from both the desired and measured
     * quaternions. The remaining roll and pitch errors are converted to a rotation vector using
     * a logarithmic map. These errors are then fed into PID controllers to compute control outputs.
     *
     * @param measuredQ the measured orientation, as a quaternion.
     * @param dt         the time step, in seconds.
     * @param rateOut    the rate setpoint for the inner loop, in deg/s.
     *
     * @note The OUTPUT of the attitude loop is a RATE, not an attitude — hence
     *       AngularRateDps. The telemetry column fed from it (att_cmd_*) is
     *       therefore one stage off from what its name suggests.
     */
    void update(const FliteQuaternion &measuredQ, float dt, AngularRateDps &rateOut);

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
    AttitudeDeg     attitudeSetpointDegs;   //< The desired orientation in degrees.
    float           deadbandRads;           //< Error deadband in radians.
    PID             pidRoll;                //< PID controller for roll.
    PID             pidPitch;               //< PID controller for pitch.
    PID             pidYaw;                 //< PID controller for yaw.

    /// Mutex to protect access to the desired orientation.
    /// Injected via setMutex(), not created here. Owning a raw FreeRTOS handle
    /// was the one thing keeping this class on the target: it is otherwise pure
    /// arithmetic, and now runs in host_sim.
    arduflite::hal::Mutex* attitudeMutex = nullptr;
};

#endif // ARDU_FLITE_ATTITUDE_CONTROLLER_H
