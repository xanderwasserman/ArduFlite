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
    constexpr PIDConfig DEFAULT_ROLL_PID  = {200.0f,  15.0f,   30.0f, 100.0f, 100.0f, 0.1f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {160.0f,  15.0f,   30.0f, 100.0f, 100.0f, 0.1f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {200.0f,  15.0f,   30.0f, 100.0f, 100.0f, 0.1f};

    constexpr float ATTITUDE_DEADBAND_RADS = 0.001f;

} // namespace AttitudeControllerConfig

namespace RateControllerConfig 
{
    constexpr PIDConfig DEFAULT_ROLL_PID  = {0.03f,    0.01f,       0.003f,     1.0f,   1.0f,   0.1f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {0.01f,    0.003f,      0.005f,     0.5f,   1.0f,   0.1f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {0.01f,    0.002f,      0.003f,     0.5f,   1.0f,   0.1f};

    constexpr float outLpAlpha = 0.01f;

} // namespace RateControllerConfig

#endif // CONTROLLER_CONFIG_H
