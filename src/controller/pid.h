#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax);

    // Updates the PID controller and returns the control output.
    float update(float error, float dt);

    // Resets the integrator and previous error.
    void reset();

    // Setters for the integrator limit and derivative filter coefficient.
    void setMaxIntegral(float max) { maxIntegral = max; }
    void setDerivativeAlpha(float alpha) { derivativeAlpha = alpha; }

private:
    float kp, ki, kd;
    float outMin, outMax;

    float integral;
    float prevError;

    float filteredDerivative = 0.0;
    float derivativeAlpha = 0.01; 
    float maxIntegral;
};

#endif // PID_H
