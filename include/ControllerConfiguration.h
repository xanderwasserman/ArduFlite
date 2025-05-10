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

namespace AttitudeControllerConfig 
{
    constexpr PIDConfig DEFAULT_ROLL_PID  = {200.0f,  0.0f,   0.0f, 60.0f, 100.0f, 0.1f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {200.0f,  0.0f,   0.0f, 60.0f, 100.0f, 0.1f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {200.0f,  0.0f,   0.0f, 60.0f, 100.0f, 0.1f};

    constexpr float ATTITUDE_DEADBAND_RADS = 0.002f;

} // namespace AttitudeControllerConfig

namespace RateControllerConfig 
{
    constexpr PIDConfig DEFAULT_ROLL_PID  = {0.015f,    0.0032f,  0.00125f,     1.0f, 0.6f, 0.03f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {0.009f,    0.0025f,  0.0036f,      1.0f, 0.6f, 0.02f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {0.015f,    0.0f,     0.00125ff,    1.0f, 0.6f, 0.03f};

    constexpr float outLpAlpha = 0.1f;

} // namespace RateControllerConfig

#endif // CONTROLLER_CONFIG_H
