#include "src/controller/ArduFliteController.h"
#include <math.h>

// Constructor: Initialize the PIDs and set the default desired orientation
ArduFliteController::ArduFliteController()
    : pidRoll(2.0f, 0.0f, 0.02f, -1.0f, 1.0f),
      pidPitch(1.0f, 0.0f, 0.05f, -1.0f, 1.0f),
      pidYaw(2.0f, 0.0f, 0.0f, -1.0f, 1.0f)
{
    // For straight and level flight (no rotation)
    desiredQ = FliteQuaternion(1, 0, 0, 0);
}

// Set the desired quaternion directly.
void ArduFliteController::setDesiredQuaternion(const FliteQuaternion &qd) {
    desiredQ = qd;
}

// Standardized function: parameters are in order: roll, pitch, yaw (all in radians).
void ArduFliteController::setDesiredEulerRads(float roll, float pitch, float yaw) {
    // Compute half-angles
    float halfRoll  = roll * 0.5f;
    float halfPitch = pitch * 0.5f;
    float halfYaw   = yaw * 0.5f;

    // Pre-compute sine and cosine of half-angles
    float cr = cos(halfRoll);
    float sr = sin(halfRoll);
    float cp = cos(halfPitch);
    float sp = sin(halfPitch);
    float cy = cos(halfYaw);
    float sy = sin(halfYaw);

    // Standard conversion (roll about x, pitch about y, yaw about z)
    desiredQ.w = cr * cp * cy + sr * sp * sy;
    desiredQ.x = sr * cp * cy - cr * sp * sy;
    desiredQ.y = cr * sp * cy + sr * cp * sy;
    desiredQ.z = cr * cp * sy - sr * sp * cy;
}

void ArduFliteController::setDesiredEulerDegs(float roll, float pitch, float yaw) {
    const float deg2rad = 3.14159265358979323846f / 180.0f;
    setDesiredEulerRads(roll * deg2rad, pitch * deg2rad, yaw * deg2rad);
}

void ArduFliteController::update(const FliteQuaternion &measuredQ, float dt, 
    float &rollOut, float &pitchOut, float &yawOut) {
    // Enforce a minimum timestep.
    if (dt < 1e-3f) dt = 1e-3f;

    // Normalize the measured quaternion.
    FliteQuaternion measuredNormalized = measuredQ;
    float normSq = measuredNormalized.normSq();
    if (normSq < 1e-6f) {
        measuredNormalized = FliteQuaternion(1, 0, 0, 0);
    }

    // Compute the error quaternion using option A:
    // qError = desiredQ * (measuredNormalized.inverse())
    FliteQuaternion qError = desiredQ * measuredNormalized.inverse();
    qError.normalize();

    // Ensure the error quaternion represents the smallest rotation.
    if (qError.w < 0) {
        qError.w = -qError.w;
        qError.x = -qError.x;
        qError.y = -qError.y;
        qError.z = -qError.z;
    }

    // Convert the error quaternion into a rotation vector using the logarithm map.
    float theta = 2.0f * acos(qError.w);
    float sinHalfTheta = sqrt(1.0f - qError.w * qError.w);
    float scale = (sinHalfTheta < 1e-6f) ? 2.0f : (theta / sinHalfTheta);

    // With standard convention, qError.x is roll error, qError.y is pitch error, qError.z is yaw error.
    float rollErr  = scale * qError.x;
    float pitchErr = scale * qError.y;
    float yawErr   = scale * qError.z;

    // Apply a deadband to filter out small noise
    float deadband = 0.02f;  // Adjust as needed (about 1.15 degrees)
    if (fabs(rollErr) < deadband)   rollErr = 0.0f;
    if (fabs(pitchErr) < deadband)  pitchErr = 0.0f;
    if (fabs(yawErr) < deadband)    yawErr = 0.0f;

    // Optionally, clamp the errors to avoid excessive commands
    float maxError = 0.5f;  // in radians; adjust based on your system
    rollErr  = constrain(rollErr, -maxError, maxError);
    pitchErr = constrain(pitchErr, -maxError, maxError);
    yawErr   = constrain(yawErr, -maxError, maxError);

    // Feed errors to the PID controllers: now a change in roll affects rollOut, pitch affects pitchOut, yaw affects yawOut.
    rollOut  = pidRoll.update(rollErr, dt);
    pitchOut = pidPitch.update(pitchErr, dt);
    yawOut   = pidYaw.update(yawErr, dt);
}

void ArduFliteController::reset() {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
}
