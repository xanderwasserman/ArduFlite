#include "src/controller/ArduFliteController.h"
#include <math.h>

// Constructor: Initialize the PIDs and set the default desired orientation
ArduFliteController::ArduFliteController()
    : pidRoll(2.0f, 0.0f, 0.01f, -1.0f, 1.0f),
      pidPitch(5.0f, 0.0f, 0.01f, -1.0f, 1.0f),
      pidYaw(2.0f, 0.0f, 0.01f, -1.0f, 1.0f)
{
    // For “straight and level” flight
    desiredQ = FliteQuaternion(1, 0, 0, 0);
}

void ArduFliteController::setDesiredQuaternion(const FliteQuaternion &qd) {
    desiredQ = qd;
}

void ArduFliteController::setDesiredEulerRads(float pitch, float roll, float yaw) {
    // Compute half-angles
    float halfRoll  = roll  * 0.5f;
    float halfPitch = pitch * 0.5f;
    float halfYaw   = yaw   * 0.5f;

    // Pre-compute sine and cosine of half-angles
    float cr = cos(halfRoll);
    float sr = sin(halfRoll);
    float cp = cos(halfPitch);
    float sp = sin(halfPitch);
    float cy = cos(halfYaw);
    float sy = sin(halfYaw);

    // Standard conversion (assuming roll about x, pitch about y, yaw about z)
    desiredQ.w = cr * cp * cy + sr * sp * sy;
    desiredQ.x = sr * cp * cy - cr * sp * sy;
    desiredQ.y = cr * sp * cy + sr * cp * sy;
    desiredQ.z = cr * cp * sy - sr * sp * cy;
}

void ArduFliteController::setDesiredEulerDegs(float pitch, float roll, float yaw) {
    const float deg2rad = 3.14159265358979323846f / 180.0f;
    setDesiredEulerRads(pitch * deg2rad, roll * deg2rad, yaw * deg2rad);
}

void ArduFliteController::update(const FliteQuaternion &measuredQ, float dt, float &rollOut, float &pitchOut, float &yawOut) {
    if (dt < 1e-3f) dt = 1e-3f;  // enforce a minimum timestep

    // Compute the error quaternion:
    // qError = measuredNormalized.inverse() * desiredQ
    FliteQuaternion measuredNormalized = measuredQ;
    float normSq = measuredNormalized.normSq();
    if (normSq < 1e-6f) {
        measuredNormalized = FliteQuaternion(1, 0, 0, 0);
    }
    // FliteQuaternion qError = measuredNormalized.inverse() * desiredQ;
    FliteQuaternion qError = desiredQ * measuredNormalized.inverse();

    // Normalize the error quaternion
    qError.normalize();

    // Convert the error quaternion into a rotation vector using the log map.
    float theta = 2.0f * acos(qError.w);
    float sinHalfTheta = sqrt(1.0f - qError.w * qError.w);
    float scale = (sinHalfTheta < 1e-6f) ? 2.0f : (theta / sinHalfTheta);
    float rollErr  = scale * qError.x;  // error about x-axis
    float pitchErr = scale * qError.y;  // error about y-axis
    float yawErr   = scale * qError.z;  // error about z-axis

    // Apply a deadband to filter out small noise
    float deadband = 0.04f;  // ~3.3 degrees (1 degree ~0.017 rad)
    if (fabs(rollErr) < deadband)   rollErr = 0.0f;
    if (fabs(pitchErr) < deadband)  pitchErr = 0.0f;
    if (fabs(yawErr) < deadband)    yawErr = 0.0f;

    // Feed errors to the PID controllers
    rollOut  = pidRoll.update(rollErr, dt);
    pitchOut = pidPitch.update(pitchErr, dt);
    yawOut   = pidYaw.update(yawErr, dt);
}

void ArduFliteController::reset() {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
}
