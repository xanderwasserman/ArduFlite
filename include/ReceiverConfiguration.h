/**
 * ReceiverConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 22 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef RECEIVER_CONFIG_H
#define RECEIVER_CONFIG_H

#include "src/receiver/pwm/ArduFlitePwmReceiver.h"
#include "include/PinConfiguration.h"

namespace ReceiverSetupConfig 
{
    constexpr ReceiverChannelConfig RECEIVER_CHANNELS[] = {
        { PwmInputConfig::ROLL_INPUT_PIN,         1000UL, 2000UL, BIPOLAR }, // Roll inpout
        { PwmInputConfig::PITCH_INPUT_PIN,        1000UL, 2000UL, BIPOLAR }, // Pitch input
        { PwmInputConfig::YAW_INPUT_PIN,          1000UL, 2000UL, BIPOLAR }, // Yaw input
        { PwmInputConfig::THROTTLE_INPUT_PIN,     1000UL, 2000UL, UNIPOLAR } // Throttle input
    };
    constexpr size_t NR_RECEIVER_CHANNELS               = sizeof(RECEIVER_CHANNELS) / sizeof(RECEIVER_CHANNELS[0]);
} // namespace ReceiverSetupConfig

#endif // RECEIVER_CONFIG_H