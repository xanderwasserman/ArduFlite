#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax);

    // Updates the PID controller and returns the control output.
    float update(float error, float dt);

    // Resets the integrator and previous error.
    void reset();

private:
    float kp, ki, kd;
    float outMin, outMax;

    float integral;
    float prevError;

    float filteredDerivative = 0.0;
    float derivativeAlpha = 0.1; 
};

#endif // PID_H
