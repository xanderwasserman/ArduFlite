// ArduFliteController.h
#ifndef ARDU_FLITE_CONTROLLER_H
#define ARDU_FLITE_CONTROLLER_H

#include "src/orientation/FliteQuaternion.h"
#include "src/controller/pid.h"

class ArduFliteController {
public:
    // You can adjust these PID gains and ranges:
    ArduFliteController()
        : pidRoll(1.0f, 0.0f, 0.01f, -1.0f, 1.0f),
          pidPitch(1.0f, 0.1f, 0.05f, -1.0f, 1.0f),
          pidYaw(0.5f, 0.0f, 0.01f, -1.0f, 1.0f)
    {
        // For “straight and level” as desired orientation:
        desiredQ = FliteQuaternion(1, 0, 0, 0);
    }

    // Set the desired orientation (as a quaternion)
    void setDesiredOrientation(const FliteQuaternion &qd) {
        desiredQ = qd;
    }

    // Or a helper to set from Euler angles if you prefer
    // (just an example, you’d need a function to build a quaternion from pitch/roll/yaw)
    // void setDesiredEuler(float pitch, float roll, float yaw);

    // Main update
    //  - measuredQ: current orientation from the IMU
    //  - dt: timestep
    //  - rollOut, pitchOut, yawOut: the final control signals in [-1..+1]
    void update(const FliteQuaternion &measuredQ, float dt, float &rollOut, float &pitchOut, float &yawOut)
    {
        if (dt < 1e-3f) dt = 1e-3f;  // Force a minimum timestep

        // qError = desiredQ * inverse(measuredQ)
        FliteQuaternion measuredNormalized = measuredQ;
        float normSq = measuredNormalized.normSq(); // assume you have this method
        if (normSq < 1e-6f) {
            // The measured quaternion is invalid; use identity (or skip this update)
            measuredNormalized = FliteQuaternion(1, 0, 0, 0);
        }
        FliteQuaternion qError = desiredQ * measuredNormalized.inverse();

        // Axis-angle form
        float rx, ry, rz, angle;
        qError.toAxisAngle(rx, ry, rz, angle);

        if (angle > M_PI) {
          angle = 2.0f*M_PI - angle;
          rx = -rx;
          ry = -ry;
          rz = -rz;
        }

        // We can interpret angle as the magnitude of the orientation error,
        // and (rx, ry, rz) as the unit axis in the body frame.
        // We'll treat each axis error as angle * axis_component. 
        // For small angles, angle ~ the combined roll/pitch/yaw error.

        float rollErr  = angle * rx;    // rotation around x is roll
        float pitchErr = angle * ry;    // rotation around y is pitch
        float yawErr   = angle * rz;    // rotation around z is yaw

        // Apply a small deadband on each error, in radians
        float deadband = 0.02f; // ~1.15 degrees (1 deg ~ 0.017 rad)
        if (fabs(rollErr) < deadband)   rollErr = 0.0f;
        if (fabs(pitchErr) < deadband)  pitchErr = 0.0f;
        if (fabs(yawErr) < deadband)    yawErr = 0.0f;

        // Now feed each error into the respective PID
        rollOut  = pidRoll.update(rollErr, dt);
        pitchOut = pidPitch.update(pitchErr, dt);
        yawOut   = pidYaw.update(yawErr, dt);
    }

    // Reset integrators if needed
    void reset() {
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
    }

private:
    FliteQuaternion desiredQ;
    PID pidRoll, pidPitch, pidYaw;
};

#endif // ARDU_FLITE_CONTROLLER_H
