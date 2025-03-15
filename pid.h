// PID.h
#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax)
      : kp(kp), ki(ki), kd(kd),
        outMin(outMin), outMax(outMax) {}

    float update(float error, float dt) {
        // Proportional
        float pTerm = kp * error;

        // Integral
        integral += error * dt;
        float iTerm = ki * integral;

        // Derivative
        float derivative = (error - prevError) / dt;
        float dTerm = kd * derivative;

        prevError = error;

        // Output
        float output = pTerm + iTerm + dTerm;

        // Clamp to outMin/outMax
        if (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;

        return output;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }

private:
    float kp, ki, kd;
    float outMin, outMax;

    float integral = 0;
    float prevError = 0;
};

#endif
