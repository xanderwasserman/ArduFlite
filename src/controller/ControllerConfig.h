#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include "src/controller/pid.h"

namespace AttitudeControllerConfig {

constexpr PIDConfig DEFAULT_ROLL_PID  = {40.0f, 0.0f, 0.05f, 45.0f, 100.0f, 0.01f};
constexpr PIDConfig DEFAULT_PITCH_PID = {40.0f, 0.0f, 0.05f, 45.0f, 100.0f, 0.01f};
constexpr PIDConfig DEFAULT_YAW_PID   = {40.0f, 0.0f, 0.05f, 45.0f, 100.0f, 0.01f};

} // namespace AttitudeControllerConfig

namespace RateControllerConfig {

constexpr PIDConfig DEFAULT_ROLL_PID  = {0.015f, 0.007f, 0.000004f, 1.0f, 100.0f, 0.01f};
constexpr PIDConfig DEFAULT_PITCH_PID = {0.015f, 0.007f, 0.000004f, 1.0f, 100.0f, 0.01f};
constexpr PIDConfig DEFAULT_YAW_PID   = {0.015f, 0.007f, 0.000004f, 1.0f, 100.0f, 0.01f};

constexpr float outLpAlpha = 0.01f;

} // namespace RateControllerConfig

#endif // CONTROLLER_CONFIG_H
