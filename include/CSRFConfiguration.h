/**
 * CRSFConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CSRF_CONFIG_H
#define CSRF_CONFIG_H

#include "src/receiver/crsf/ArdufliteCRSFReceiver.h"
#include "src/receiver/crsf/ArdufliteCRSFCallbacks.h"

/**
 * Static array of ChannelConfig for all 16 CRSF channels.
 * 
 * Edit *only* this array to re-map sticks, switches, etc.
 */

namespace CRSFConfig
{
    // Inline so you can #include in multiple translation units without ODR
    inline ChannelConfig CRSF_CHANNEL_CONFIGS[16] = {
        // ch 1 = roll: bipolar stick → –1…+1
        { .type      = ChannelType::DualThrow,
        .callback  = onRoll },

        // ch 2 = pitch: bipolar stick → –1…+1
        { .type      = ChannelType::DualThrow,
        .callback  = onPitch },

        //ch 3 = throttle
        {}, 

        // ch 4 = yaw: bipolar stick → 0…+1
        { .type     = ChannelType::DualThrow,
        .callback   = onYaw },

        // ch 5 = mode switch: boolean (off/on)
        { .type     = ChannelType::Boolean,
        .callback   = onModeSwitch },

        // ch 6 = mode switch: boolean (off/on)
        { .type     = ChannelType::Boolean,
        .callback   = onActivateMission },

        // ch 7 = mission switch: boolean (off/on)
        { .type     = ChannelType::Boolean,
        .callback   = onArm },

        /*
        // ch 4 = flight-mode switch: tri-state (–1,0,+1)
        { .type      = ChannelType::TriState,
        .thrLow    = 0.4f,
        .thrHigh   = 0.6f,
        .callback  = [](uint8_t ch, float v){
            // inline lambda is OK too
            // …handle mode…
        }},
        */

        // ch 9–15 = leave as Raw (0…2047) with no callback:
        {},{},{},{},{},{},{},{},{}
    };
} 

#endif //CSRF_CONFIG_H