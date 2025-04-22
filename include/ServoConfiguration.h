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
    constexpr ServoConfig PITCH_CONFIG      = { PwmOutputConfig::PITCH_PIN,      500, 2500, 90, 70, false };
    constexpr ServoConfig YAW_CONFIG        = { PwmOutputConfig::YAW_PIN,        500, 2500, 90, 70, false };
    constexpr ServoConfig LEFT_AIL_CONFIG   = { PwmOutputConfig::LEFT_AIL_PIN,   500, 2500, 90, 70, true };
    constexpr ServoConfig RIGHT_AIL_CONFIG  = { PwmOutputConfig::RIGHT_AIL_PIN,  500, 2500, 90, 70, false };

} // namespace ServoSetupConfig

#endif // SERVO_CONFIG_H