#include <math.h>
#include <Arduino.h>
#include "src/controller/ArduFliteAttitudeController.h"

// ---------------------------------------------------------------------------
// ArduFliteAttitudeController Class Implementation
// ---------------------------------------------------------------------------

/**
 * @brief Constructor: Initializes the PID controllers and sets the default
 * desired orientation to "straight and level" (no rotation).
 */
 ArduFliteAttitudeController::ArduFliteAttitudeController()
    : pidRoll(  1.0f,   0.0f,   0.05f,  -45.0f,     45.0f),
      pidPitch( 1.0f,   0.0f,   0.05f,  -45.0f,     45.0f),
      pidYaw(   2.0f,   0.0f,   0.0f,   -45.0f,     45.0f)
{
    // Desired orientation is no rotation.
    desiredQ = FliteQuaternion(1, 0, 0, 0);

    // Create the mutex for protecting desiredQ.
    attitudeMutex = xSemaphoreCreateMutex();
    if (attitudeMutex == NULL) {
        Serial.println("Failed to create ArduFliteAttitudeController mutex!");
    }
}

/**
 * @brief Sets the desired orientation directly as a quaternion.
 *
 * @param qd The desired quaternion.
 */
void ArduFliteAttitudeController::setDesiredQuaternion(const FliteQuaternion &qd) {
    xSemaphoreTake(attitudeMutex, portMAX_DELAY);
    desiredQ = qd;
    xSemaphoreGive(attitudeMutex);
}

/**
 * @brief Sets the desired orientation using Euler angles in radians.
 *
 * The parameters are provided in the order: roll, pitch, yaw. This function
 * uses the standard aerospace convention: roll about x, pitch about y, yaw about z.
 *
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw   Yaw angle in radians.
 */
void ArduFliteAttitudeController::setDesiredEulerRads(float roll, float pitch, float yaw) {
    // Compute half-angles.
    float halfRoll  = roll  * 0.5f;
    float halfPitch = pitch * 0.5f;
    float halfYaw   = yaw   * 0.5f;

    // Pre-compute sine and cosine for efficiency.
    float cr = cos(halfRoll);
    float sr = sin(halfRoll);
    float cp = cos(halfPitch);
    float sp = sin(halfPitch);
    float cy = cos(halfYaw);
    float sy = sin(halfYaw);

    // Convert Euler angles to a quaternion using the standard formula.
    FliteQuaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    // Use the setter that protects desiredQ.
    setDesiredQuaternion(q);
}

/**
 * @brief Sets the desired orientation using Euler angles in degrees.
 *
 * @param roll  Roll angle in degrees.
 * @param pitch Pitch angle in degrees.
 * @param yaw   Yaw angle in degrees.
 */
void ArduFliteAttitudeController::setDesiredEulerDegs(float roll, float pitch, float yaw) {
    const float deg2rad = PI / 180.0f;
    setDesiredEulerRads(roll * deg2rad, pitch * deg2rad, yaw * deg2rad);
}

/**
 * @brief Updates the controller based on the measured orientation.
 *
 * This function decouples the yaw error from the roll and pitch errors. It:
 * 1. Extracts and computes yaw error using a wrapped angle difference.
 * 2. Removes the yaw component from both the desired and measured quaternions.
 * 3. Computes the error quaternion for roll and pitch.
 * 4. Converts the roll/pitch error quaternion to a rotation vector using the log map.
 * 5. Feeds the computed errors to the respective PID controllers.
 *
 * @param measuredQ The measured orientation as a quaternion.
 * @param dt        The time step in seconds.
 * @param rollOut   The controller output for roll (to be applied to the ailerons).
 * @param pitchOut  The controller output for pitch (to be applied to the elevators).
 * @param yawOut    The controller output for yaw (to be applied to the rudder).
 */
void ArduFliteAttitudeController::update(const FliteQuaternion &measuredQ, float dt, float &rollOut, float &pitchOut, float &yawOut) {
    // Enforce a minimum timestep to prevent division by zero.
    if (dt < 1e-3f) dt = 1e-3f;

    // Read desiredQ in a thread-safe way.
    FliteQuaternion localDesiredQ;
    xSemaphoreTake(attitudeMutex, portMAX_DELAY);
    localDesiredQ = desiredQ;
    xSemaphoreGive(attitudeMutex);

    // Normalize the measured quaternion.
    FliteQuaternion measuredNormalized = measuredQ;
    float normSq = measuredNormalized.normSq();
    if (normSq < 1e-6f) {
        measuredNormalized = FliteQuaternion(1, 0, 0, 0);
    }

    // --- Compute Yaw Error Separately ---
    float desiredYaw = extractYaw(localDesiredQ);
    float measuredYaw = extractYaw(measuredNormalized);
    float yawErr = wrapAngle(desiredYaw - measuredYaw);

    // --- Remove Yaw from Both Desired and Measured Quaternions ---
    FliteQuaternion desiredNoYaw = removeYaw(localDesiredQ);
    FliteQuaternion measuredNoYaw = removeYaw(measuredNormalized);

    // --- Compute Roll/Pitch Error Quaternion ---
    // qErrorRP represents the rotation needed (ignoring yaw) to go from the measured
    // to the desired orientation.
    FliteQuaternion qErrorRP = desiredNoYaw * measuredNoYaw.inverse();
    qErrorRP.normalize();
    // Ensure the error quaternion represents the smallest rotation.
    if (qErrorRP.w < 0) {
        qErrorRP.w = -qErrorRP.w;
        qErrorRP.x = -qErrorRP.x;
        qErrorRP.y = -qErrorRP.y;
        qErrorRP.z = -qErrorRP.z;
    }

    // --- Convert the Error Quaternion to a Rotation Vector (Log Map) ---
    float theta = 2.0f * acos(qErrorRP.w);
    float sinHalfTheta = sqrt(1.0f - qErrorRP.w * qErrorRP.w);
    float scale = (sinHalfTheta < 1e-6f) ? 2.0f : (theta / sinHalfTheta);
    // In the standard convention, qErrorRP.x is the roll error and qErrorRP.y is the pitch error.
    float rollErr  = scale * qErrorRP.x;
    float pitchErr = scale * qErrorRP.y;
    
    // --- Apply Deadband to Filter Out Small Noise ---
    float deadband = 0.01f;  // Adjust as needed (about 0.57 degrees)
    if (fabs(rollErr) < deadband)   rollErr = 0.0f;
    if (fabs(pitchErr) < deadband)  pitchErr = 0.0f;
    if (fabs(yawErr) < deadband)    yawErr = 0.0f;

    // --- Feed the Errors to the PID Controllers ---
    rollOut  = pidRoll.update(rollErr, dt);
    pitchOut = pidPitch.update(pitchErr, dt);
    yawOut   = pidYaw.update(yawErr, dt);
}

/**
 * @brief Resets all PID controllers.
 */
void ArduFliteAttitudeController::reset() {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
}

// ---------------------------------------------------------------------------
// Helper Functions for Decoupling Yaw
// ---------------------------------------------------------------------------

/**
 * @brief Extracts the yaw angle (rotation about Z) from a quaternion.
 * 
 * Uses the standard conversion formula:
 * yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
 * 
 * @param q The input quaternion.
 * @return float The yaw angle in radians.
 */
static float extractYaw(const FliteQuaternion &q) {
    return atan2(2.0f * (q.w * q.z + q.x * q.y),
                 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

/**
 * @brief Wraps an angle to the interval [-π, π].
 * 
 * @param angle The input angle in radians.
 * @return float The wrapped angle.
 */
static float wrapAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

/**
 * @brief Removes the yaw component from a quaternion.
 * 
 * This is achieved by constructing a quaternion that represents the
 * inverse of the yaw rotation and multiplying it with the input quaternion.
 * 
 * @param q The input quaternion.
 * @return FliteQuaternion The quaternion with yaw removed.
 */
static FliteQuaternion removeYaw(const FliteQuaternion &q) {
    float yaw = extractYaw(q);
    float halfYaw = -yaw * 0.5f;
    // Create a quaternion that undoes the yaw rotation.
    FliteQuaternion yawInv(cos(halfYaw), 0, 0, sin(halfYaw));
    return yawInv * q; // Assuming operator* implements quaternion multiplication.
}
