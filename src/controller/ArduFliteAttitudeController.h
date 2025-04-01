#ifndef ARDU_FLITE_ATTITUDE_CONTROLLER_H
#define ARDU_FLITE_ATTITUDE_CONTROLLER_H

#include "src/orientation/FliteQuaternion.h"
#include "src/controller/pid.h"

static float extractYaw(const FliteQuaternion &q);
static float wrapAngle(float angle);
static FliteQuaternion removeYaw(const FliteQuaternion &q);

class ArduFliteAttitudeController {
public:
    // Constructor
    ArduFliteAttitudeController();

    // Set the desired orientation as a quaternion
    void setDesiredQuaternion(const FliteQuaternion &qd);

    // Set the desired orientation from Euler angles (radians)
    void setDesiredEulerRads(float pitch, float roll, float yaw);

    // Set the desired orientation from Euler angles (degrees)
    void setDesiredEulerDegs(float pitch, float roll, float yaw);

    // Main update function:
    // measuredQ: current orientation from the IMU
    // dt: timestep
    // rollOut, pitchOut, yawOut: final control signals in [-1..+1]
    void update(const FliteQuaternion &measuredQ, float dt, float &rollOut, float &pitchOut, float &yawOut);

    // Reset the PID integrators
    void reset();

private:
    FliteQuaternion desiredQ;
    PID pidRoll, pidPitch, pidYaw;

    // Mutex for protecting access to class state.
    SemaphoreHandle_t attitudeMutex;
};

#endif // ARDU_FLITE_ATTITUDE_CONTROLLER_H
