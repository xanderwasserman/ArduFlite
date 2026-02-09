/**
 * ConfigHelpers.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 07 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Helper functions for building runtime configuration structures.
 *        Converts stored Ti/Td parameters to Ki/Kd for PID controllers.
 */
#ifndef CONFIG_HELPERS_H
#define CONFIG_HELPERS_H

#include "src/controller/pid.h"
#include "src/utils/ConfigRegistry.h"
#include <Arduino.h>

namespace ConfigHelpers {

/**
 * @brief Calculate maximum integral term for anti-windup.
 * 
 * @param outLimit Maximum output limit
 * @param ki Integral gain
 * @param headroom Anti-windup headroom factor (0.0-1.0)
 * @return Maximum integral value
 */
inline float calcMaxIntegral(float outLimit, float ki, float headroom = 0.9f) {
    return (ki > 0.0f ? (outLimit / ki) * headroom : 0.0f);
}

/**
 * @brief Build a PIDConfig from stored Ti/Td parameters.
 * 
 * Converts time-constant form (Kp, Ti, Td) to gain form (Kp, Ki, Kd):
 *   Ki = Kp / Ti (if Ti > 0)
 *   Kd = Kp * Td
 * 
 * @param keyPrefix Key prefix (e.g., "rate.roll" or "att.pitch")
 * @return PIDConfig with computed gains
 */
inline PIDConfig buildPIDConfig(const char* keyPrefix) {
    char key[32];
    auto& reg = ConfigRegistry::instance();
    
    // Build keys for each parameter
    snprintf(key, sizeof(key), "%s.kp", keyPrefix);
    float kp = reg.get<float>(key);
    
    snprintf(key, sizeof(key), "%s.ti", keyPrefix);
    float ti = reg.get<float>(key);
    
    snprintf(key, sizeof(key), "%s.td", keyPrefix);
    float td = reg.get<float>(key);
    
    snprintf(key, sizeof(key), "%s.outlimit", keyPrefix);
    float outLimit = reg.get<float>(key);
    
    snprintf(key, sizeof(key), "%s.headroom", keyPrefix);
    float headroom = reg.get<float>(key);
    
    snprintf(key, sizeof(key), "%s.alpha", keyPrefix);
    float alpha = reg.get<float>(key);
    
    // Convert Ti/Td to Ki/Kd
    float ki = (ti > 0.0f) ? (kp / ti) : 0.0f;
    float kd = kp * td;
    float maxI = calcMaxIntegral(outLimit, ki, headroom);
    
    return PIDConfig{ kp, ki, kd, outLimit, maxI, alpha };
}

/**
 * @brief Build a PIDConfig from stored Ti/Td parameters (with explicit keys).
 * 
 * Use this overload when keys don't follow the standard pattern.
 * 
 * @param kpKey Key for Kp parameter
 * @param tiKey Key for Ti parameter
 * @param tdKey Key for Td parameter
 * @param outLimitKey Key for output limit
 * @param headroomKey Key for anti-windup headroom
 * @param alphaKey Key for derivative filter alpha
 * @return PIDConfig with computed gains
 */
inline PIDConfig buildPIDConfigExplicit(
    const char* kpKey,
    const char* tiKey,
    const char* tdKey,
    const char* outLimitKey,
    const char* headroomKey,
    const char* alphaKey
) {
    auto& reg = ConfigRegistry::instance();
    
    float kp = reg.get<float>(kpKey);
    float ti = reg.get<float>(tiKey);
    float td = reg.get<float>(tdKey);
    float outLimit = reg.get<float>(outLimitKey);
    float headroom = reg.get<float>(headroomKey);
    float alpha = reg.get<float>(alphaKey);
    
    // Convert Ti/Td to Ki/Kd
    float ki = (ti > 0.0f) ? (kp / ti) : 0.0f;
    float kd = kp * td;
    float maxI = calcMaxIntegral(outLimit, ki, headroom);
    
    return PIDConfig{ kp, ki, kd, outLimit, maxI, alpha };
}

} // namespace ConfigHelpers

#endif // CONFIG_HELPERS_H
