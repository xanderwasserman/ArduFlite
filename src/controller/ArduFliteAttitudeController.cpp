/**
 * ArduFliteAttitudeController.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @file ArduFliteAttitudeController.cpp
 * @brief Implements the attitude (outer loop) controller for ArduFlite.
 *
 * This controller computes the desired angular rates based on the error
 * between the current measured orientation and the desired orientation.
 * It uses PID controllers for roll, pitch, and yaw, and employs a mutex
 * (attitudeMutex) to protect the shared desired orientation (desiredQ).
 */
 #include "src/controller/ArduFliteAttitudeController.h"
#include "src/utils/Logging.h"
#include "src/utils/ConfigHelpers.h"
#include "include/ConfigKeys.h"

 #include <math.h>
 #include <cmath>

// Forward declarations for static helper functions defined later in this file.
static float extractYaw(const FliteQuaternion &q);
static float wrapAngle(float angle);
static FliteQuaternion removeYaw(const FliteQuaternion &q, float yaw);
static AttitudeDeg quaternionToAttitudeDeg(const FliteQuaternion &q);

 /**
  * @brief Default constructor.
  *
  * Creates mutex but does NOT initialize PIDs. PIDs are zero-initialized
  * and must be configured by calling initFromConfig() after ConfigRegistry is ready.
  */
 ArduFliteAttitudeController::ArduFliteAttitudeController()
    // Declaration order: members initialise in that order regardless of what
    // is written here, so the two are kept matched.
    : deadbandRads(0.0001f), pidRoll(), pidPitch(), pidYaw()
 {
     // Desired orientation is initialized to no rotation.
     desiredQ = FliteQuaternion(1, 0, 0, 0);

     // The mutex arrives via setMutex(), from Board's pool. See the note in
     // ArduFliteRateController's constructor.
 }

 /**
  * @brief Initialize PID controllers from ConfigRegistry.
  *        Must be called after ConfigRegistry::init().
  */
void ArduFliteAttitudeController::initFromConfig()
{
    std::unique_lock lock(*attitudeMutex);
    if (!lock.owns_lock()) {
        LOG_ERR("AttitudeController: failed to acquire mutex during init");
        return;
    }

    pidRoll.setConfig(ConfigHelpers::buildPIDConfig(CONFIG_KEY_ATT_ROLL_PREFIX,  "outlimit_dps"));
    pidPitch.setConfig(ConfigHelpers::buildPIDConfig(CONFIG_KEY_ATT_PITCH_PREFIX, "outlimit_dps"));
    // pidYaw is intentionally NOT configured here. Without a magnetometer there is no
    // fixed heading reference, so yaw is passed through from the pilot setpoint directly
    // in update(). pidYaw remains as a placeholder for future magnetometer integration.
    deadbandRads = ConfigRegistry::instance().get<float>(CONFIG_KEY_ATT_DEADBAND_RAD);

    LOG_INF("AttitudeController: initialized from ConfigRegistry");
}

 /**
  * @brief Sets the desired orientation directly from a quaternion.
  *
  * Normalizes qd, derives the equivalent Euler setpoint, and writes BOTH desiredQ and
  * attitudeSetpointDegs atomically under attitudeMutex. The two must stay in sync because
  * update() takes yaw from attitudeSetpointDegs.yaw (the magnetometer-less yaw passthrough),
  * so writing desiredQ alone would leave yaw stale. This is the natural entry point for a
  * future AHRS / GPS-compass heading source that produces a full orientation quaternion.
  *
  * @param qd The desired orientation as a quaternion (need not be unit-norm).
  */
 void ArduFliteAttitudeController::setAttitudeControlSetpointQuaternion(const FliteQuaternion &qd)
 {
    // Work on a unit quaternion: update() consumes desiredQ without re-normalizing, and the
    // Euler extraction below assumes unit norm.
    FliteQuaternion q = qd;
    q.normalize();

    // Derive the matching Euler setpoint so attitudeSetpointDegs tracks desiredQ.
    const AttitudeDeg setpointDegs = quaternionToAttitudeDeg(q);

    {
        std::unique_lock lock(*attitudeMutex);
        if (!lock.owns_lock()) return;
        attitudeSetpointDegs = setpointDegs;
        desiredQ             = q;
    }
 }

 /**
  * @brief Sets the desired orientation using Euler angles in degrees.
  *
  * Builds the desired quaternion and writes both desiredQ and attitudeSetpointDegs
  * atomically under attitudeMutex. This is the primary setpoint path used by the controller.
  *
  * @param setpointDegs  Attitude Setpoint in degrees.
  */
 void ArduFliteAttitudeController::setAttitudeControlSetpoint(AttitudeDeg setpointDegs)
 {
    // Compute quaternion BEFORE acquiring the lock so both fields are written
    // atomically in a single critical section (eliminates TOCTOU window between
    // attitudeSetpointDegs and desiredQ that update() could observe as a mismatch).
    //
    // Store setpointDegs directly rather than delegating to the quaternion setter: a
    // degs→quat→degs round-trip would fold yaw to ±180° and lose the exact pilot yaw at
    // pitch ±90° (gimbal lock). Convert from the representation we were handed.
    const float deg2rad = PI / 180.0f;
    const float halfRoll  = setpointDegs.roll  * deg2rad * 0.5f;
    const float halfPitch = setpointDegs.pitch * deg2rad * 0.5f;
    const float halfYaw   = setpointDegs.yaw   * deg2rad * 0.5f;
    const float cr = cosf(halfRoll),  sr = sinf(halfRoll);
    const float cp = cosf(halfPitch), sp = sinf(halfPitch);
    const float cy = cosf(halfYaw),   sy = sinf(halfYaw);
    FliteQuaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    {
        std::unique_lock lock(*attitudeMutex);
        if (!lock.owns_lock()) return;
        attitudeSetpointDegs = setpointDegs;
        desiredQ             = q;
    }
 }

 /**
  * @brief Updates the attitude controller.
  *
  * This method computes the control error between the desired and measured orientations,
  * decouples yaw from roll and pitch, converts the roll/pitch error quaternion into
  * a rotation vector using a logarithmic map, applies a deadband to filter out noise,
  * and then feeds the resulting errors into the respective PID controllers to obtain
  * control outputs for roll, pitch, and yaw.
  *
  * @param measuredQ The measured orientation as a quaternion.
  * @param dt        Time step in seconds.
  * @param rateOut   (Output) Control output for rates.
  */
 void ArduFliteAttitudeController::update(const FliteQuaternion &measuredQ, float dt, AngularRateDps &rateOut)
 {
    // Prevent a too-small timestep.
    if (dt < 1e-3f) dt = 1e-3f;

    // Retrieve desired orientation in a thread-safe manner.
    FliteQuaternion localDesiredQ;
    AttitudeDeg     localAttitudeSetpointDegs;
    float           deadband;

    {
        std::unique_lock lock(*attitudeMutex);
        if (!lock.owns_lock()) return;
        localDesiredQ = desiredQ;
        localAttitudeSetpointDegs = attitudeSetpointDegs;
        deadband = deadbandRads;
    }

    // Normalize the measured quaternion.
    FliteQuaternion measuredNormalized = measuredQ;
    float normSq = measuredNormalized.normSq();
    if (normSq < 1e-6f) {
        LOG_WARN("AttitudeController: near-zero quaternion (normSq=%.2e) — using identity. Check IMU.", normSq);
        measuredNormalized = FliteQuaternion(1, 0, 0, 0);
    } else {
        measuredNormalized.normalize();
    }

    // --- Compute Yaw Error Separately ---
    float desiredYaw = extractYaw(localDesiredQ);
    float measuredYaw = extractYaw(measuredNormalized);
    float yawErr = wrapAngle(desiredYaw - measuredYaw);

    // --- Remove Yaw from Both Quaternions ---
    // Pass the pre-computed yaw values so extractYaw() is not called a second time.
    FliteQuaternion desiredNoYaw  = removeYaw(localDesiredQ,       desiredYaw);
    FliteQuaternion measuredNoYaw = removeYaw(measuredNormalized,  measuredYaw);

    // --- Compute Roll/Pitch Error Quaternion ---
    FliteQuaternion qErrorRP = desiredNoYaw * measuredNoYaw.inverse();
    qErrorRP.normalize();
    // Ensure the error quaternion represents the smallest rotation.
    if (qErrorRP.w < 0) {
        qErrorRP.w = -qErrorRP.w;
        qErrorRP.x = -qErrorRP.x;
        qErrorRP.y = -qErrorRP.y;
        qErrorRP.z = -qErrorRP.z;
    }
    // Clamp w to the valid acosf domain [-1, 1] to prevent NaN from float drift.
    qErrorRP.w = constrain(qErrorRP.w, -1.0f, 1.0f);

    // --- Convert Error Quaternion to Rotation Vector (Log Map) ---
    float theta = 2.0f * acosf(qErrorRP.w);
    float sinHalfTheta = sqrtf(1.0f - qErrorRP.w * qErrorRP.w);
    float scale = (sinHalfTheta < 1e-6f) ? 2.0f : (theta / sinHalfTheta);
    float rollErr  = scale * qErrorRP.x;   // Roll error component.
    float pitchErr = scale * qErrorRP.y;   // Pitch error component.

    // --- Apply Deadband to Filter Out Noise ---
    if (fabsf(rollErr) < deadband)   rollErr = 0.0f;
    if (fabsf(pitchErr) < deadband)  pitchErr = 0.0f;
    if (fabsf(yawErr) < deadband)    yawErr = 0.0f;

    // --- Feed Errors to PID Controllers ---
    {
        std::unique_lock lock(*attitudeMutex);
        if (!lock.owns_lock()) return;
        rateOut.roll  = pidRoll.update(rollErr, dt);
        rateOut.pitch = pidPitch.update(pitchErr, dt);
        // Yaw is passed through from the pilot setpoint: no heading reference
        // without a magnetometer. pidYaw remains for future magnetometer support.
        rateOut.yaw = localAttitudeSetpointDegs.yaw;
    }
 }

 /**
  * @brief Resets all PID controllers.
  *
  * This method resets the integrators and derivative states for the roll, pitch,
  * and yaw PID controllers.
  */
 void ArduFliteAttitudeController::reset()
 {
     std::unique_lock lock(*attitudeMutex);
     if (!lock.owns_lock()) return;
     pidRoll.reset();
     pidPitch.reset();
     pidYaw.reset();
 }

 void ArduFliteAttitudeController::resetIntegrals()
 {
     std::unique_lock lock(*attitudeMutex);
     if (!lock.owns_lock()) return;
     pidRoll.resetIntegral();
     pidPitch.resetIntegral();
     pidYaw.resetIntegral();
 }

// ─────────────────────────────────────────────────────────────────
// Runtime Configuration Updates
// ─────────────────────────────────────────────────────────────────

void ArduFliteAttitudeController::setPIDConfig(ControlLoopType loop, const PIDConfig& config)
{
    std::unique_lock lock(*attitudeMutex);
    if (!lock.owns_lock()) return;

    switch (loop) {
        case ATTITUDE_ROLL_LOOP:
            pidRoll.setConfig(config);
            break;
        case ATTITUDE_PITCH_LOOP:
            pidPitch.setConfig(config);
            break;
        case ATTITUDE_YAW_LOOP:
            pidYaw.setConfig(config);
            break;
        default:
            LOG_WARN("Invalid loop type for attitude PID config: %d", loop);
            break;
    }
}

void ArduFliteAttitudeController::setDeadband(float deadband)
{
    std::unique_lock lock(*attitudeMutex);
    if (!lock.owns_lock()) return;
    deadbandRads = deadband;
}

 /*============================================================================
   Helper Functions for Decoupling Yaw
   ============================================================================*/

 /**
  * @brief Extracts the yaw angle (rotation about Z) from a quaternion.
  *
  * Uses the standard conversion formula:
  * yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
  *
  * @param q The input quaternion.
  * @return float The yaw angle in radians.
  */
 static float extractYaw(const FliteQuaternion &q)
 {
     return atan2f(2.0f * (q.w * q.z + q.x * q.y),
                   1.0f - 2.0f * (q.y * q.y + q.z * q.z));
 }

 /**
  * @brief Extracts roll, pitch, and yaw (radians) from a unit quaternion.
  *
  * Standard body 3-2-1 (yaw-pitch-roll) Tait-Bryan extraction — the exact inverse of the
  * quaternion construction in setAttitudeControlSetpoint(). Reuses extractYaw() for the
  * yaw term so the heading formula lives in one place.
  *
  * @param q A unit quaternion (caller is responsible for normalizing).
  * @return roll/pitch/yaw in DEGREES.
  *
  * @note The trigonometry below is inherently in radians, and the conversion is
  *       folded in here on purpose: radians then never cross a function
  *       boundary anywhere in the flight layer, so no radians TYPE is needed
  *       (Phase 6B). The intermediate is a plain local with a _rad suffix,
  *       which is the correct carrier for a scalar (ADR-038).
  */
 static AttitudeDeg quaternionToAttitudeDeg(const FliteQuaternion &q)
 {
     const float roll_rad  = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                                    1.0f - 2.0f * (q.x * q.x + q.y * q.y));
     // Clamp the pitch argument to asinf's domain to prevent NaN from float drift near ±90°.
     const float sinPitch   = constrain(2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f);
     const float pitch_rad  = asinf(sinPitch);
     const float yaw_rad    = extractYaw(q);

     constexpr float kRadToDeg = 180.0f / PI;
     return AttitudeDeg{ roll_rad * kRadToDeg, pitch_rad * kRadToDeg, yaw_rad * kRadToDeg };
 }

 /**
  * @brief Wraps an angle to the interval [-π, π].
  *
  * @param angle The input angle in radians.
  * @return float The wrapped angle.
  */
 static float wrapAngle(float angle)
 {
     // O(1) arithmetic wrap to [-π, π] — avoids unbounded loops at startup.
     angle -= TWO_PI * floorf((angle + PI) / TWO_PI);
     return angle;
 }

 /**
  * @brief Removes the yaw component from a quaternion.
  *
  * Constructs a quaternion that represents the inverse of the yaw rotation
  * and multiplies it with the input quaternion to remove the yaw.
  *
  * @param q   The input quaternion.
  * @param yaw Pre-computed yaw angle in radians (from extractYaw(q)).
  * @return FliteQuaternion The quaternion with yaw removed.
  */
 static FliteQuaternion removeYaw(const FliteQuaternion &q, float yaw)
 {
     float halfYaw = -yaw * 0.5f;
     // Create a quaternion that undoes the yaw rotation.
     FliteQuaternion yawInv(cosf(halfYaw), 0.0f, 0.0f, sinf(halfYaw));
     return yawInv * q; // Assumes operator* is defined for quaternion multiplication.
 }

