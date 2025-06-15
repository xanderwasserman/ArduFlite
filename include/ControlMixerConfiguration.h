/**
 * ControlMixerConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 11 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Compile-time options & limits for ControlMixer.
 */
#ifndef CONTROL_MIXER_CONFIG_H
#define CONTROL_MIXER_CONFIG_H

// ——— Feature flags ———
#define ENABLE_MIXING              ///< SAFE-style mixing in ATTITUDE_MODE

// ——— Attitude limits (degrees) ———
constexpr float MAX_ATT_ROLL  =  45.0f;  ///< ± roll angle in ATTITUDE_MODE
constexpr float MAX_ATT_PITCH =  45.0f;  ///< ± pitch angle
constexpr float MAX_ATT_YAW   = 180.0f;  ///< ± yaw heading offset

// ——— Rate limits (°/s) ———
constexpr float MAX_RATE_ROLL  = 90.0f; ///< ± roll rate in RATE_MODE
constexpr float MAX_RATE_PITCH = 60.0f; ///< ± pitch rate
constexpr float MAX_RATE_YAW   = 60.0f; ///< ± yaw rate

// ——— Mixing coefficients ———
constexpr float MIX_ATT_ROLL_FROM_YAW   = 0.00f; ///< yaw→roll 
constexpr float MIX_ATT_PITCH_FROM_ROLL = 0.05f; ///< roll→pitch
constexpr float MIX_ATT_YAW_FROM_ROLL   = 0.10f; ///< roll→yaw

#endif // CONTROL_MIXER_CONFIG_H