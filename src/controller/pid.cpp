#include "pid.h"
#include <algorithm> // for std::max and std::min

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
    : kp(kp), ki(ki), kd(kd), outMin(outMin), outMax(outMax),
      integral(0.0f), prevError(0.0f)
{
}

float PID::update(float error, float dt) {
    // Proportional term
    float pTerm = kp * error;

    // Candidate integral update
    float newIntegral = integral + error * dt;
    float iTermCandidate = ki * newIntegral;

    // Derivative term
    float derivative = (error - prevError) / dt;
    float dTerm = kd * derivative;

    // Compute the unsaturated output
    float unsatOutput = pTerm + iTermCandidate + dTerm;

    // Saturate the output
    float satOutput = unsatOutput;
    if (unsatOutput > outMax) {
        satOutput = outMax;
    } else if (unsatOutput < outMin) {
        satOutput = outMin;
    }

    // Anti-windup: update the integrator only if:
    // 1) The unsaturated output is within limits, or
    // 2) The output is saturated, but the current error would reduce the saturation.
    if ((unsatOutput >= outMin && unsatOutput <= outMax) ||
        ((satOutput == outMax) && (error < 0)) ||
        ((satOutput == outMin) && (error > 0))) {
        integral = newIntegral;
    }
    // Otherwise, we do not update the integrator to prevent windup.

    prevError = error;
    return satOutput;
}

void PID::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}
