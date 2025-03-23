#include "pid.h"

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
    : kp(kp), ki(ki), kd(kd), outMin(outMin), outMax(outMax),
      integral(0.0f), prevError(0.0f)
{
}

float PID::update(float error, float dt) {
    // Proportional term
    float pTerm = kp * error;

    // Integral term
    integral += error * dt;
    float iTerm = ki * integral;

    // Derivative term
    float derivative = (error - prevError) / dt;
    float dTerm = kd * derivative;

    prevError = error;

    // Sum all contributions
    float output = pTerm + iTerm + dTerm;

    // Clamp to specified output limits
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    return output;
}

void PID::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}
