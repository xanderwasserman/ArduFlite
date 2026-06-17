/**
 * ArduFliteRateController.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/controller/ArduFliteRateController.h"
#include "src/orientation/ArduFliteIMU.h"
#include "include/ArduFlite.h"
#include "src/utils/Logging.h"
#include "src/utils/ConfigHelpers.h"
#include "include/ConfigKeys.h"

// Default constructor - creates mutex but does NOT initialize PIDs.
// Call initFromConfig() after ConfigRegistry is ready.
ArduFliteRateController::ArduFliteRateController()
    : pidRoll(), pidPitch(), pidYaw(), outputAlpha(0.1f)
{
    rateMutex = xSemaphoreCreateMutex();
    if (rateMutex == NULL) {
        LOG_ERR("Failed to create ArduFliteRateController mutex!");
    }
}

// Initialize PID controllers from ConfigRegistry.
void ArduFliteRateController::initFromConfig()
{
    SemaphoreLock lock(rateMutex);
    if (!lock.acquired()) {
        LOG_ERR("RateController: failed to acquire mutex during init");
        return;
    }

    pidRoll.setConfig(ConfigHelpers::buildPIDConfig(CONFIG_KEY_RATE_ROLL_PREFIX));
    pidPitch.setConfig(ConfigHelpers::buildPIDConfig(CONFIG_KEY_RATE_PITCH_PREFIX));
    pidYaw.setConfig(ConfigHelpers::buildPIDConfig(CONFIG_KEY_RATE_YAW_PREFIX));
    // Range is enforced by the ConfigRegistry schema ([0.001, 1.0]); no clamp needed here.
    outputAlpha = ConfigRegistry::instance().get<float>(CONFIG_KEY_RATE_OUT_LP_ALPHA);

    LOG_INF("RateController: initialized from ConfigRegistry");
}

// Set the desired angular rates (e.g., from the outer loop's output)
void ArduFliteRateController::setRateControlSetpoint(const EulerAngles &setpoint)
{
    // Protect the update to desired rates.
    {
        SemaphoreLock lock(rateMutex, 0);
        if (!lock.acquired()) return;
        setpointRate = setpoint;
    }
}

// Update the rate controller with the measured angular rates.
// The error is computed as (desiredRate - measuredRate) for each axis.
void ArduFliteRateController::update(Vector3 measuredRate, float dt, EulerAngles &actuatorOut)
{
    // Prevent a too-small dt.
    if (dt < 1e-3f) dt = 1e-3f;

    SemaphoreLock lock(rateMutex, 0);
    if (!lock.acquired()) return;

    // Compute errors while holding rateMutex. The PID instances, output filter,
    // and runtime config setters share this lock.
    float rollError  = setpointRate.roll  - measuredRate.x;
    float pitchError = setpointRate.pitch - measuredRate.y;
    float yawError   = setpointRate.yaw   - measuredRate.z;

    // Compute raw PID outputs.
    float newRollOut  = pidRoll.update(rollError, dt);
    float newPitchOut = pidPitch.update(pitchError, dt);
    float newYawOut   = pidYaw.update(yawError, dt);

    // Guard the EMA against NaN/Inf. A non-finite PID output (e.g. from a NaN
    // setpoint) would latch in filteredRateOutput permanently, since NaN
    // propagates through every subsequent EMA step. Drop the bad sample and
    // hold the last good filter state instead.
    if (!isfinite(newRollOut) || !isfinite(newPitchOut) || !isfinite(newYawOut))
    {
        LOG_ERR("RateController: NaN/Inf in PID output — holding last command.");
        actuatorOut.roll  = constrain(filteredRateOutput.roll,  -1.0f, 1.0f);
        actuatorOut.pitch = constrain(filteredRateOutput.pitch, -1.0f, 1.0f);
        actuatorOut.yaw   = constrain(filteredRateOutput.yaw,   -1.0f, 1.0f);
        return;
    }

    // Apply an exponential moving average filter to smooth the output.
    filteredRateOutput.roll  = outputAlpha * newRollOut  + (1.0f - outputAlpha) * filteredRateOutput.roll;
    filteredRateOutput.pitch = outputAlpha * newPitchOut + (1.0f - outputAlpha) * filteredRateOutput.pitch;
    filteredRateOutput.yaw   = outputAlpha * newYawOut   + (1.0f - outputAlpha) * filteredRateOutput.yaw;

    // Final clamp on the output copy: bound the servo command to [-1, 1]. Note this
    // clamps actuatorOut only, not the EMA state itself — with bounded PID output
    // (PID saturates at outLimit) the filter state stays in range in normal operation.
    actuatorOut.roll  = constrain(filteredRateOutput.roll,  -1.0f, 1.0f);
    actuatorOut.pitch = constrain(filteredRateOutput.pitch, -1.0f, 1.0f);
    actuatorOut.yaw   = constrain(filteredRateOutput.yaw,   -1.0f, 1.0f);
}

// Reset all PID controllers.
void ArduFliteRateController::reset()
{
    SemaphoreLock lock(rateMutex);
    if (!lock.acquired()) return;
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    // Clear the output EMA state too — otherwise the stale pre-disarm command
    // bleeds into the filter on (re-)arm, causing a brief servo transient.
    filteredRateOutput = {0.0f, 0.0f, 0.0f};
}

void ArduFliteRateController::resetIntegrals()
{
    SemaphoreLock lock(rateMutex, 0);
    if (!lock.acquired()) return;
    pidRoll.resetIntegral();
    pidPitch.resetIntegral();
    pidYaw.resetIntegral();
}

// ─────────────────────────────────────────────────────────────────
// Runtime Configuration Updates
// ─────────────────────────────────────────────────────────────────

void ArduFliteRateController::setPIDConfig(ControlLoopType loop, const PIDConfig& config)
{
    SemaphoreLock lock(rateMutex);
    if (!lock.acquired()) return;

    switch (loop) {
        case RATE_ROLL_LOOP:
            pidRoll.setConfig(config);
            break;
        case RATE_PITCH_LOOP:
            pidPitch.setConfig(config);
            break;
        case RATE_YAW_LOOP:
            pidYaw.setConfig(config);
            break;
        default:
            LOG_WARN("Invalid loop type for rate PID config: %d", loop);
            break;
    }
}

void ArduFliteRateController::setOutputAlpha(float alpha)
{
    SemaphoreLock lock(rateMutex);
    if (!lock.acquired()) return;
    // Caller is responsible for range — the only caller sources this from the
    // ConfigRegistry, whose schema already validates it to [0.001, 1.0].
    outputAlpha = alpha;
}
