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

namespace AttitudeControllerConfig {

    constexpr PIDConfig DEFAULT_ROLL_PID  = {40.0f, 0.04f, 0.05f, 45.0f, 100.0f, 0.01f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {40.0f, 0.04f, 0.05f, 45.0f, 100.0f, 0.01f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {40.0f, 0.04f, 0.05f, 45.0f, 100.0f, 0.01f};

} // namespace AttitudeControllerConfig

namespace RateControllerConfig {

    constexpr PIDConfig DEFAULT_ROLL_PID  = {0.025f, 0.008f, 0.000005f, 1.0f, 100.0f, 0.01f};
    constexpr PIDConfig DEFAULT_PITCH_PID = {0.025f, 0.008f, 0.000005f, 1.0f, 100.0f, 0.01f};
    constexpr PIDConfig DEFAULT_YAW_PID   = {0.025f, 0.008f, 0.000005f, 1.0f, 100.0f, 0.01f};

    constexpr float outLpAlpha = 0.01f;

} // namespace RateControllerConfig

#endif // CONTROLLER_CONFIG_H
