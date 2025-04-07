#ifndef ARDU_FLITE_ATTITUDE_CONTROLLER_H
#define ARDU_FLITE_ATTITUDE_CONTROLLER_H

#include "src/orientation/FliteQuaternion.h"
#include "src/controller/pid.h"
#include <Arduino.h>

/**
 * @brief Extracts the yaw angle (rotation about the Z-axis) from a quaternion.
 *
 * This function uses the standard conversion formula:
 * @f[
 * yaw = \arctan\left(\frac{2(wz + xy)}{1 - 2(y^2 + z^2)}\right)
 * @f]
 *
 * @param q The input quaternion.
 * @return float The yaw angle in radians.
 */
static float extractYaw(const FliteQuaternion &q);

/**
 * @brief Wraps an angle to the interval [-π, π].
 *
 * @param angle The input angle in radians.
 * @return float The angle wrapped to [-π, π].
 */
static float wrapAngle(float angle);

/**
 * @brief Removes the yaw component from a quaternion.
 *
 * This function constructs a quaternion that represents the inverse of the yaw rotation
 * and multiplies it with the input quaternion, effectively removing the yaw component.
 *
 * @param q The input quaternion.
 * @return FliteQuaternion The quaternion with the yaw removed.
 */
static FliteQuaternion removeYaw(const FliteQuaternion &q);

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
     * @brief Constructor.
     *
     * Initializes the PID controllers for roll, pitch, and yaw with preset gains and output
     * limits, sets the default desired orientation to "straight and level" (i.e., no rotation),
     * and creates a mutex for protecting the desired orientation.
     */
    ArduFliteAttitudeController();

    /**
     * @brief Sets the desired orientation using a quaternion.
     *
     * Updates the internal desired orientation in a thread-safe manner.
     *
     * @param qd The desired orientation as a quaternion.
     */
    void setDesiredQuaternion(const FliteQuaternion &qd);

    /**
     * @brief Sets the desired orientation using Euler angles in radians.
     *
     * Converts the provided Euler angles (roll, pitch, yaw) to a quaternion and updates
     * the desired orientation.
     *
     * @param pitch Pitch angle in radians.
     * @param roll Roll angle in radians.
     * @param yaw Yaw angle in radians.
     */
    void setDesiredEulerRads(float pitch, float roll, float yaw);

    /**
     * @brief Sets the desired orientation using Euler angles in degrees.
     *
     * Converts the provided Euler angles (roll, pitch, yaw) from degrees to radians and
     * updates the desired orientation.
     *
     * @param pitch Pitch angle in degrees.
     * @param roll Roll angle in degrees.
     * @param yaw Yaw angle in degrees.
     */
    void setDesiredEulerDegs(float pitch, float roll, float yaw);

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
    void update(const FliteQuaternion &measuredQ, float dt, float &rollOut, float &pitchOut, float &yawOut);

    /**
     * @brief Resets the PID controllers.
     *
     * Resets the integrators and derivative states for the roll, pitch, and yaw PID controllers.
     */
    void reset();

private:
    FliteQuaternion desiredQ; ///< The desired orientation.
    PID pidRoll;              ///< PID controller for roll.
    PID pidPitch;             ///< PID controller for pitch.
    PID pidYaw;               ///< PID controller for yaw.

    /// Mutex to protect access to the desired orientation.
    SemaphoreHandle_t attitudeMutex;
};

#endif // ARDU_FLITE_ATTITUDE_CONTROLLER_H
