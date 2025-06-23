/**
 * ControllerConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 12 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include "src/controller/pid.h"

/**
 *  Anti-windup clamp: keep the integral term within ±(outLimit/Ki)*headroom.
 */
constexpr float calcMaxIntegral(float outLimit, float ki, float headroom = 0.9f) {
    return (ki > 0.0f ? (outLimit / ki) * headroom : 0.0f);
}

/**
 *  Factory: given Kp, Ti (s), Td (s), outLimit, headroom and alpha,
 *  compute Ki = Kp/Ti, Kd = Kp*Td, clamp = calcMaxIntegral(), and pack into PIDConfig.
 */
constexpr PIDConfig makePID_TC(
    float Kp,            // proportional gain
    float Ti,            // integral time constant [s]
    float Td,            // derivative time constant [s]
    float outLimit,      // ± output limit
    float headroom = 0.8f, // anti-windup headroom fraction
    float alpha = 0.1f     // derivative filter (manually tuned)
) {
    float Ki = (Ti > 0.0f ? Kp / Ti : 0.0f);
    float Kd = Kp * Td;
    float maxI = calcMaxIntegral(outLimit, Ki, headroom);
    return { Kp, Ki, Kd, outLimit, maxI, alpha };
}

/*
Increase Ti if you see bias / steady-state error.

Decrease Ti if you see windup / overshoot.

Increase Td if you need more damping.

Decrease Td if you see derivative‐driven noise.
*/

namespace AttitudeControllerConfig
{
    constexpr PIDConfig DEFAULT_ROLL_PID  = makePID_TC(320.00f,  0.00f,   0.00f,    90.00f,   0.80f,   0.10f);
    constexpr PIDConfig DEFAULT_PITCH_PID = makePID_TC(200.00f,  0.00f,   0.00f,    60.00f,   0.80f,   0.10f);
    constexpr PIDConfig DEFAULT_YAW_PID   = makePID_TC(200.00f,  0.00f,   0.00f,    60.00f,   0.80f,   0.10f);

    constexpr float ATTITUDE_DEADBAND_RADS = 0.0001f;
}

namespace RateControllerConfig
{
    constexpr PIDConfig DEFAULT_ROLL_PID  = makePID_TC(0.09f,   1.40f,   0.30f,     1.00f,   0.80f,   0.10f);
    constexpr PIDConfig DEFAULT_PITCH_PID = makePID_TC(0.06f,   5.00f,   0.70f,     1.00f,   0.80f,   0.10f);
    constexpr PIDConfig DEFAULT_YAW_PID   = makePID_TC(0.05f,   3.00f,   0.30f,     1.00f,   0.80f,   0.10f);

    constexpr float outLpAlpha = 0.005f;  // separate low-pass for actuator output
}

#endif // CONTROLLER_CONFIG_H
