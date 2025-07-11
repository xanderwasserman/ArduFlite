/**
 * ArduFliteAttitudeController.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
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
#include "include/ArduFlite.h"

 #include <math.h>
 #include <Arduino.h>
 
 /**
  * @brief Constructor.
  *
  * Initializes the PID controllers with predetermined gains and output limits,
  * sets the default desired orientation to "straight and level" (no rotation),
  * and creates the mutex to protect access to the desired orientation.
  */
 ArduFliteAttitudeController::ArduFliteAttitudeController()
    : pidRoll(AttitudeControllerConfig::DEFAULT_ROLL_PID),
      pidPitch(AttitudeControllerConfig::DEFAULT_PITCH_PID),
      pidYaw(AttitudeControllerConfig::DEFAULT_YAW_PID)
 {
     // Desired orientation is initialized to no rotation.
     desiredQ = FliteQuaternion(1, 0, 0, 0);
 
     // Create the mutex for protecting desiredQ.
     attitudeMutex = xSemaphoreCreateMutex();
     if (attitudeMutex == NULL) {
         LOG_ERR("Failed to create ArduFliteAttitudeController mutex!");
     }
 }
 
 /**
  * @brief Sets the desired orientation directly.
  *
  * This method updates the desired orientation (desiredQ) in a thread-safe manner.
  *
  * @param qd The desired orientation as a quaternion.
  */
 void ArduFliteAttitudeController::setAttitudeControlSetpointQuaternion(const FliteQuaternion &qd) 
 {
    {
        SemaphoreLock lock(attitudeMutex);
        desiredQ = qd;
    }
 }
 
 /**
  * @brief Sets the desired orientation using Euler angles in radians.
  *
  * This method converts the provided Euler angles (roll, pitch, yaw) into a
  * quaternion (using standard aerospace conventions) and sets the desired
  * orientation.
  *
  * @param setpointRads Attitude setpoint in radians.
  */
 void ArduFliteAttitudeController::setAttitudeControlSetpointRads(EulerAngles setpointRads) 
 { 
    // Compute half-angles.
    float halfRoll  = setpointRads.roll  * 0.5f;
    float halfPitch = setpointRads.pitch * 0.5f;
    float halfYaw   = setpointRads.yaw   * 0.5f;

    // Pre-compute sine and cosine for efficiency.
    float cr = cos(halfRoll);
    float sr = sin(halfRoll);
    float cp = cos(halfPitch);
    float sp = sin(halfPitch);
    float cy = cos(halfYaw);
    float sy = sin(halfYaw);

    // Convert Euler angles to a quaternion.
    FliteQuaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    // Set the desired orientation with thread protection.
    setAttitudeControlSetpointQuaternion(q);
 }
 
 /**
  * @brief Sets the desired orientation using Euler angles in degrees.
  *
  * This method converts degrees to radians and calls setAttitudeControlSetpointRads().
  *
  * @param setpointDegs  Attitude Setpoint in degrees.
  */
 void ArduFliteAttitudeController::setAttitudeControlSetpoint(EulerAngles setpointDegs) 
 {
    {
        SemaphoreLock lock(attitudeMutex);
        attitudeSetpointDegs       = setpointDegs;
    }
    
    const float         deg2rad = PI / 180.0f;
    EulerAngles         setpointRad;

    setpointRad.roll    = setpointDegs.roll * deg2rad;
    setpointRad.pitch   = setpointDegs.pitch * deg2rad;
    setpointRad.yaw     = setpointDegs.yaw * deg2rad;

    setAttitudeControlSetpointRads(setpointRad);
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
 void ArduFliteAttitudeController::update(const FliteQuaternion &measuredQ, float dt, EulerAngles &rateOut) 
 {
    // Prevent a too-small timestep.
    if (dt < 1e-3f) dt = 1e-3f;

    // Retrieve desired orientation in a thread-safe manner.
    FliteQuaternion localDesiredQ;
    EulerAngles     localAttitudeSetpointDegs;

    {
        SemaphoreLock lock(attitudeMutex);
        localDesiredQ = desiredQ;
        localAttitudeSetpointDegs = attitudeSetpointDegs;
    }

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

    // --- Remove Yaw from Both Quaternions ---
    FliteQuaternion desiredNoYaw = removeYaw(localDesiredQ);
    FliteQuaternion measuredNoYaw = removeYaw(measuredNormalized);

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

    // --- Convert Error Quaternion to Rotation Vector (Log Map) ---
    float theta = 2.0f * acos(qErrorRP.w);
    float sinHalfTheta = sqrt(1.0f - qErrorRP.w * qErrorRP.w);
    float scale = (sinHalfTheta < 1e-6f) ? 2.0f : (theta / sinHalfTheta);
    float rollErr  = scale * qErrorRP.x;   // Roll error component.
    float pitchErr = scale * qErrorRP.y;   // Pitch error component.
    
    // --- Apply Deadband to Filter Out Noise ---
    float deadband = AttitudeControllerConfig::ATTITUDE_DEADBAND_RADS; 
    if (fabs(rollErr) < deadband)   rollErr = 0.0f;
    if (fabs(pitchErr) < deadband)  pitchErr = 0.0f;
    if (fabs(yawErr) < deadband)    yawErr = 0.0f;

    // --- Feed Errors to PID Controllers ---
    rateOut.roll  = pidRoll.update(rollErr, dt);
    rateOut.pitch = pidPitch.update(pitchErr, dt);
    //  rateOut.yaw   = pidYaw.update(yawErr, dt);
    rateOut.yaw = localAttitudeSetpointDegs.yaw; //! we just pass on the yaw setpoint to the rate controller, as we have no fixed heading reference (no magnetometer).
 }
 
 /**
  * @brief Resets all PID controllers.
  *
  * This method resets the integrators and derivative states for the roll, pitch,
  * and yaw PID controllers.
  */
 void ArduFliteAttitudeController::reset() 
 {
     pidRoll.reset();
     pidPitch.reset();
     pidYaw.reset();
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
     return atan2(2.0f * (q.w * q.z + q.x * q.y),
                  1.0f - 2.0f * (q.y * q.y + q.z * q.z));
 }
 
 /**
  * @brief Wraps an angle to the interval [-π, π].
  *
  * @param angle The input angle in radians.
  * @return float The wrapped angle.
  */
 static float wrapAngle(float angle) 
 {
     while (angle > PI) angle -= TWO_PI;
     while (angle < -PI) angle += TWO_PI;
     return angle;
 }
 
 /**
  * @brief Removes the yaw component from a quaternion.
  *
  * Constructs a quaternion that represents the inverse of the yaw rotation
  * and multiplies it with the input quaternion to remove the yaw.
  *
  * @param q The input quaternion.
  * @return FliteQuaternion The quaternion with yaw removed.
  */
 static FliteQuaternion removeYaw(const FliteQuaternion &q) 
 {
     float yaw = extractYaw(q);
     float halfYaw = -yaw * 0.5f;
     // Create a quaternion that undoes the yaw rotation.
     FliteQuaternion yawInv(cos(halfYaw), 0, 0, sin(halfYaw));
     return yawInv * q; // Assumes operator* is defined for quaternion multiplication.
 }
 