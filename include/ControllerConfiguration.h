/**
 * ControllerConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include "src/controller/pid.h"

namespace AttitudeControllerConfig 
{
    constexpr PIDConfig DEFAULT_ROLL_PID  = {240.0f,  0.0f,   0.0f, 60.0f, 100.0f, 0.1f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {240.0f,  0.0f,   0.0f, 60.0f, 100.0f, 0.1f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {240.0f,  0.0f,   0.0f, 60.0f, 100.0f, 0.1f};

} // namespace AttitudeControllerConfig

namespace RateControllerConfig 
{
    constexpr PIDConfig DEFAULT_ROLL_PID  = {0.017f,  0.0024f,  0.00125f,   1.0f, 0.2f, 0.15f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {0.017f,  0.002f,   0.0015f,    1.0f, 0.2f, 0.15f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {0.015f,  0.0f,     0.0f,       1.0f, 0.2f, 0.15f};

    constexpr float outLpAlpha = 0.8f;

} // namespace RateControllerConfig

#endif // CONTROLLER_CONFIG_H
