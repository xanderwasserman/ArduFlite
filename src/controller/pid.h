#ifndef PID_H
#define PID_H

#include <algorithm>

/**
 * @brief Structure to hold the PID controller configuration.
 */
struct PIDConfig {
    float kp;              ///< Proportional gain.
    float ki;              ///< Integral gain.
    float kd;              ///< Derivative gain.
    float outLimit;        ///< Max output magnitude (saturation upper and lower bound).
    float maxIntegral;     ///< Maximum allowed integral value (anti-windup).
    float derivativeAlpha; ///< Low-pass filter coefficient for the derivative.
};

class PID {
public:
    /**
     * @brief Constructs a PID controller using the provided configuration.
     * 
     * @param cfg The configuration structure containing all tunable PID parameters.
     */
    PID(const PIDConfig& cfg);

    /**
     * @brief Updates the PID controller.
     * 
     * @param error The current error value.
     * @param dt Time step in seconds.
     * @return The computed control output.
     */
    float update(float error, float dt);

    /**
     * @brief Resets the integrator and previous error.
     */
    void reset();

    // Setters to update the configuration at runtime.
    void setConfig(const PIDConfig &cfg) { config = cfg; }
    void setMaxIntegral(float max) { config.maxIntegral = max; }
    void setDerivativeAlpha(float alpha) { config.derivativeAlpha = alpha; }

private:
    PIDConfig config;   ///< Holds all the tunable PID configuration parameters.
    
    float integral              = 0.0f; ///< Accumulated integral error.
    float prevError             = 0.0f; ///< Previous error (for derivative calculation).
    float filteredDerivative    = 0.0f; ///< Derivative term, low-pass filtered.
};

#endif // PID_H
