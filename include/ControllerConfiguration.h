/**
 * ControllerConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include "src/controller/pid.h"

constexpr float calcMaxIntegral(float outLimit, float ki, float headroom = 0.9f) {
    // give 90% of the theoretical maximum
    return (ki > 0.0f ? (outLimit / ki) * headroom : 0.0f);
}

constexpr PIDConfig makePID(
    float kp,
    float ki,
    float kd,
    float outLimit,
    float headroomFactor = 0.8f,
    float alpha = 0.1f
) {
    return {
        kp,
        ki,
        kd,
        outLimit,
        calcMaxIntegral(outLimit, ki, headroomFactor),
        alpha
    };
}

//rate_setpoint [°/s] =  Kp [°/s per rad] × angle_error [rad]  +  Ki [°/s per (rad·s)] × ∫error·dt  +  Kd [… per (rad/s)] × d(error)/dt
namespace AttitudeControllerConfig { 
    // now you only declare (kp, ki, kd, outLimit, headroom, alpha)
    constexpr PIDConfig DEFAULT_ROLL_PID  = makePID(    200.0f,     15.0f,  30.0f,      60.0f,     0.8f,   0.1f);
    constexpr PIDConfig DEFAULT_PITCH_PID = makePID(    160.0f,     15.0f,  30.0f,      60.0f,     0.8f,   0.1f);
    constexpr PIDConfig DEFAULT_YAW_PID   = makePID(    200.0f,     15.0f,  30.0f,      60.0f,     0.8f,   0.1f);

    constexpr float ATTITUDE_DEADBAND_RADS = 0.001f;
}

//actuator_cmd [–1…1] =  Kp [1/(°/s)] × rate_error [°/s]  +  Ki […/((°/s)·s)] × ∫rate_error·dt  +  …
namespace RateControllerConfig { 
    constexpr PIDConfig DEFAULT_ROLL_PID  = makePID(    0.03f,      0.01f,   0.003f,     1.0f,  0.8f,   0.1f);
    constexpr PIDConfig DEFAULT_PITCH_PID = makePID(    0.01f,      0.003f,  0.005f,     1.0f,  0.8f,   0.1f);
    constexpr PIDConfig DEFAULT_YAW_PID   = makePID(    0.01f,      0.002f,  0.003f,     1.0f,  0.8f,   0.1f);

    constexpr float outLpAlpha = 0.01f;
}

#endif // CONTROLLER_CONFIG_H
