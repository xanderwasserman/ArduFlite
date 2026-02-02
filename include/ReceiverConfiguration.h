/**
 * ReceiverConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 22 April 2025
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

// ─────────────────────────────────────────────────────────────────────────────
// Failsafe Configuration
// ─────────────────────────────────────────────────────────────────────────────
namespace FailsafeConfig
{
    /// Bank angle during failsafe spiral descent (degrees).
    /// Gentle bank (5-7°) keeps spiral contained without aggressive descent.
    constexpr float FAILSAFE_BANK_DEG       = 7.0f;
    
    /// Pitch angle during failsafe (degrees, negative = nose down).
    /// Slight nose-down maintains airspeed for controlled glide.
    constexpr float FAILSAFE_PITCH_DEG      = -3.0f;
    
    /// Throttle setting during failsafe (0.0 = cut, 1.0 = full).
    /// We cut throttle to prevent runaway and reduce fire risk on impact.
    constexpr float FAILSAFE_THROTTLE       = 0.0f;
    
    /// Minimum link quality (0-100%) required to arm.
    constexpr uint8_t MIN_LINK_QUALITY_ARM  = 50;

} // namespace FailsafeConfig

#endif // RECEIVER_CONFIG_H
