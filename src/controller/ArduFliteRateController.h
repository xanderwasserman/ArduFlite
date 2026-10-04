/**
 * ArduFliteRateController.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDU_FLITE_RATE_CONTROLLER_H
#define ARDU_FLITE_RATE_CONTROLLER_H

#include <mutex>

#include "src/hal/platform/Mutex.h"

#include "src/controller/pid.h"
#include "include/ControllerTypes.h"
#include "src/core/FlightTypes.h"

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
    /// Supply the lock guarding this controller's state. Must be called before
    /// any other method; every one of them becomes a no-op until it is.
    void setMutex(arduflite::hal::Mutex* mutex) { rateMutex = mutex; }

    /**
     * @brief Default constructor with uninitialized PIDs.
     *        Call initFromConfig() before use.
     */
    ArduFliteRateController();

    /**
     * @brief Initialize PID controllers from ConfigRegistry.
     *        Must be called after ConfigRegistry::init().
     */
    void initFromConfig();

    // Set the desired angular rates (roll, pitch, yaw). Units can be degrees per second.
    void setRateControlSetpoint(const AngularRateDps& setpoint);

    // Main update function:
    //   measuredRate: measured angular rates from the IMU (x=roll, y=pitch, z=yaw).
    //   dt:           time step in seconds.
    //   actuatorOut:  final control signals (normalized to [-1, 1]) to drive the servos.
    //
    // Uses a non-blocking lock: if the mutex is contended this call returns early
    // WITHOUT modifying actuatorOut, so the caller's previous command is held. The
    // caller must persist actuatorOut across iterations to rely on this fail-soft.
    /**
     * @param measuredRate gyro reading, deg/s, body frame
     * @param actuatorOut  normalised -1..+1 per-axis demand for the mixer
     *
     * @note The types differ because the quantities differ: a rate goes in, a
     *       dimensionless command comes out. The PID gains are what carry the
     *       conversion, which is why there is no named converter for this step.
     */
    void update(const AngularRateDps& measuredRate, float dt, AxisCommand& actuatorOut);

    // Reset the PID controllers' integrators.
    void reset();

    /**
     * @brief Resets only the integral accumulators for all rate PIDs.
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
     * @param loop The control loop type (RATE_ROLL_LOOP, RATE_PITCH_LOOP, RATE_YAW_LOOP)
     * @param config The PID configuration
     */
    void setPIDConfig(ControlLoopType loop, const PIDConfig& config);

    /**
     * @brief Set the output low-pass filter alpha.
     * @param alpha Filter alpha (0.0-1.0)
     */
    void setOutputAlpha(float alpha);

private:
    // Desired angular rates (set by the outer loop)
    AngularRateDps setpointRate{};
    AxisCommand    filteredRateOutput{};
    float outputAlpha               = 0.1f;

    // PID controllers for each axis.
    PID pidRoll, pidPitch, pidYaw;

    // Mutex for protecting access to class state.
    /// Injected via setMutex(), not created here. Owning a raw FreeRTOS handle
    /// was the one thing keeping this class on the target: it is otherwise pure
    /// arithmetic, and now runs in host_sim.
    arduflite::hal::Mutex* rateMutex = nullptr;
};

#endif // ARDU_FLITE_RATE_CONTROLLER_H
