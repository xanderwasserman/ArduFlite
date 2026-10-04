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
 * @param keyPrefix       key prefix, e.g. "rate.roll" or "att.pitch"
 * @param outLimitSuffix  the output-limit key's suffix, WITHOUT a leading dot.
 *                        It differs per loop and must: the rate loop's limit is
 *                        a dimensionless -1..+1 surface command ("outlimit"),
 *                        the attitude loop's is a rate in deg/s
 *                        ("outlimit_dps"). A wrong suffix resolves to no key,
 *                        which silently yields a limit of ZERO — a PID that
 *                        clamps its own output to nothing.
 * @return PIDConfig with computed gains
 */
inline PIDConfig buildPIDConfig(const char* keyPrefix, const char* outLimitSuffix = "outlimit") {
    char key[32];
    auto& reg = ConfigRegistry::instance();
    
    // Build keys for each parameter
    snprintf(key, sizeof(key), "%s.kp", keyPrefix);
    float kp = reg.get<float>(key);
    
    // "_s" — these are TIME constants in seconds, and Phase 1's unit-suffix
    // rename (schema v2) renamed the schema keys without updating this
    // composer. The registry then reported "key not found" and handed back a
    // default of 0 for every PID's integral and derivative term, on every
    // controller, silently turning them into pure-P loops.
    snprintf(key, sizeof(key), "%s.ti_s", keyPrefix);
    float ti = reg.get<float>(key);
    
    snprintf(key, sizeof(key), "%s.td_s", keyPrefix);
    float td = reg.get<float>(key);
    
    snprintf(key, sizeof(key), "%s.%s", keyPrefix, outLimitSuffix);
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
