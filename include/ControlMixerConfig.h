/**
 * ControlMixerConfig.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 09 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONTROL_MIXER_CONFIG_H
#define CONTROL_MIXER_CONFIG_H

// In ATTITUDE_MODE only: mix yaw & pitch into roll (SAFE-style)
#define ENABLE_SAFE_MIXING

// Lets you individually enable/disable roll/pitch/yaw outputs for tuning.
// #define ENABLE_SINGLE_AXIS_CONTROL

#endif // CONTROL_MIXER_CONFIG_H