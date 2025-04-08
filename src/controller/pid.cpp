/**
 * pid.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "pid.h"

PID::PID(const PIDConfig& cfg): config(cfg)
{
}

float PID::update(float error, float dt) {
    if (dt < 1e-3f)
        dt = 1e-3f;

    // Proportional term.
    float pTerm = config.kp * error;

    // Update and clamp the integral term to prevent windup.
    float newIntegral = integral + error * dt;
    newIntegral = std::clamp(newIntegral, -config.maxIntegral, config.maxIntegral);
    float iTermCandidate = config.ki * newIntegral;

    // Derivative term with low-pass filtering.
    float derivative = (error - prevError) / dt;
    filteredDerivative = config.derivativeAlpha * derivative +
                         (1.0f - config.derivativeAlpha) * filteredDerivative;
    float dTerm = config.kd * filteredDerivative;

    // Compute the unsaturated output.
    float unsatOutput = pTerm + iTermCandidate + dTerm;
    // Saturate the output.
    float satOutput = std::clamp(unsatOutput, -config.outLimit, config.outLimit);

    // Anti-windup logic: Update the integral only if either output is not saturated
    // or the error is driving the output back into the unsaturated range.
    if ((unsatOutput >= -config.outLimit && unsatOutput <= config.outLimit) ||
        ((satOutput == config.outLimit) && (error < 0)) ||
        ((satOutput == -config.outLimit) && (error > 0))) {
        integral = newIntegral;
    }
    prevError = error;
    return satOutput;
}

void PID::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}
