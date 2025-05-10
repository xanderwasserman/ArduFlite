/**
 * ServoConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 21 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>

#include "include/PinConfiguration.h"
#include "src/actuators/ServoManager.h"

namespace ServoSetupConfig 
{
    constexpr float MAX_SERVO_DEG_PER_SEC = 300.0f;

#if BOARD_TYPE == BOARD_TYPE_FIREBEETLE
    constexpr ServoConfig PITCH_CONFIG      = { PwmOutputConfig::PITCH_PIN,      500, 2500, 90, 70, false };
    constexpr ServoConfig YAW_CONFIG        = { PwmOutputConfig::YAW_PIN,        500, 2500, 90, 70, false };
    constexpr ServoConfig LEFT_AIL_CONFIG   = { PwmOutputConfig::LEFT_AIL_PIN,   500, 2500, 90, 70, true };
    constexpr ServoConfig RIGHT_AIL_CONFIG  = { PwmOutputConfig::RIGHT_AIL_PIN,  500, 2500, 90, 70, false };
#elif BOARD_TYPE == BOARD_TYPE_WEMOS
    constexpr ServoConfig PITCH_CONFIG      = { PwmOutputConfig::PITCH_PIN,      500, 2500, 90, 70, true };
    constexpr ServoConfig YAW_CONFIG        = { PwmOutputConfig::YAW_PIN,        500, 2500, 90, 70, true };
    constexpr ServoConfig LEFT_AIL_CONFIG   = { PwmOutputConfig::LEFT_AIL_PIN,   500, 2500, 90, 70, true };
    constexpr ServoConfig RIGHT_AIL_CONFIG  = { PwmOutputConfig::RIGHT_AIL_PIN,  500, 2500, 90, 70, false };
#endif
} // namespace ServoSetupConfig

#endif // SERVO_CONFIG_H