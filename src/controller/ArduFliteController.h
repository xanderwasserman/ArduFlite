#ifndef ARDU_FLITE_CONTROLLER_H
#define ARDU_FLITE_CONTROLLER_H

#include "src/orientation/FliteQuaternion.h"
#include "src/controller/pid.h"

class ArduFliteController {
public:
    // Constructor
    ArduFliteController();

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
};

#endif // ARDU_FLITE_CONTROLLER_H
