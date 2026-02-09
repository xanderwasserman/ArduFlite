/**
 * ServoManager.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <Arduino.h>
#include <ESP32Servo.h>

// Wing design types.
enum WingDesign {
    CONVENTIONAL, // Separate elevator, rudder, and aileron(s)
    DELTA_WING,   // Two elevons (mix pitch and roll)
    V_TAIL        // Two ruddervators (mix pitch and yaw)
};

// Servo configuration structure.
struct ServoConfig {
    int pin;         // Digital pin for the servo.
    int minPulse;    // Minimum pulse width in microseconds.
    int maxPulse;    // Maximum pulse width in microseconds.
    int neutral;     // Neutral (center) angle (typically ~90).
    int deflection;  // Maximum deflection from neutral (e.g., 70 maps [-1,1] to [-70,70]).
    bool invert;     // Inversion flag to flip the direction.
};

class ServoManager {
public:
    /**
     * @brief Default constructor for deferred initialization.
     * 
     * Creates an uninitialized ServoManager. Call initFromConfig() after
     * ConfigRegistry is ready to load configuration and attach servos.
     */
    ServoManager();

    /**
     * @brief Initialize from ConfigRegistry.
     * 
     * Reads servo configuration (pins, pulses, deflection, etc.) from
     * ConfigRegistry and attaches all servos. Must be called after
     * ConfigRegistry::init().
     */
    void initFromConfig();

    /**
     * @brief Write control commands to the servos.
     *
     * The meaning of the control inputs changes with wing design:
     * - CONVENTIONAL:
     *     • pitchCmd maps to the elevator.
     *     • yawCmd maps to the rudder.
     *     • rollCmd maps to aileron deflection.
     * - DELTA_WING:
     *     • pitchCmd and rollCmd are mixed to form elevon deflections.
     * - V_TAIL:
     *     • pitchCmd and yawCmd are mixed to form ruddervator deflections.
     *
     * All command inputs are expected in the range [-1, +1].
     */
    void writeCommands(float rollCmd, float pitchCmd, float yawCmd);

    /**
     * @brief Write throttle commands to the ESC.
     *
     * Command input is expected in the range [0, 1.0].
     */
    void writeThrottle(float throttleCmd);

    /**
     * @brief Runs a self-test routine by wiggling control surfaces.
     *
     * Moves the control surfaces to their full positive, full negative, and neutral positions
     * in sequence over multiple cycles, ensuring the servo outputs respond correctly.
     * This is useful for verifying hardware functionality during startup or in the field.
     */
    void testControlSurfaces();

    /** 
     * @brief Set maximum servo slew rate in degrees per second.
     *        e.g. 300 means no servo moves faster than 300°/s.
     */
    void setMaxServoRate(float degPerSec) { maxServoDegPerSec = degPerSec; }

    /** 
     * @brief Set maximum throttle slew rate in full-range per second.
     */
    void setMaxThrottleRate(float rate) { maxThrottlePerSec = rate; }

    // Setter methods to adjust inversion flags at runtime.
    void setPitchInversion(bool invert);
    void setYawInversion(bool invert);
    void setLeftSurfaceInversion(bool invert);
    void setRightSurfaceInversion(bool invert);

    // (Optional) Setter methods to update entire configurations.
    void setPitchConfig(const ServoConfig &config);
    void setYawConfig(const ServoConfig &config);
    void setLeftSurfaceConfig(const ServoConfig &config);
    void setRightSurfaceConfig(const ServoConfig &config);

private:
    // Helper: maps a float value from one range to another.
    static int mapFloatToInt(float val, float inMin, float inMax, int outMin, int outMax);

    // Wing design type.
    WingDesign wingDesign;

    // For conventional wing designs.
    ServoConfig pitchConfig;
    ServoConfig yawConfig;
    ServoConfig leftAilConfig;      // In conventional, this is the aileron (or single aileron).
    ServoConfig rightAilConfig;     // Only used if dualAilerons is true.
    ServoConfig throttleConfig;
    bool        dualAilerons;       // True if using two separate aileron servos.

    // For delta wing or V-tail designs, we use these two surfaces.
    // (For conventional designs these are the aileron servos.)
    // We will refer to them as the left and right surfaces.
    // In delta wing, they serve as elevons; in V-tail, as ruddervators.
    
    // Servo objects.
    // For conventional design:
    Servo pitchServo, yawServo;         // Elevator and rudder.
    Servo singleAilServo;               // Used if dualAilerons is false.
    Servo leftAilServo, rightAilServo;  // Used if dualAilerons is true.
    Servo throttleServo;                // Throttle ESC.

    // slew‑rate limiter state (last output angles in degrees)
    float lastPitchAngleDeg  = 0.0f;
    float lastYawAngleDeg    = 0.0f;
    float lastLeftAngleDeg   = 0.0f;
    float lastRightAngleDeg  = 0.0f;
    float lastSingleAilDeg   = 0.0f;
    float lastThrottleCmd    = 0.0f;

    // limiter configuration
    float maxServoDegPerSec         = 300.0f;   // default cap: 300°/s
    float maxThrottlePerSec         = 1.0f;     // full range per second
    unsigned long lastUpdateMicros  = 0;        // timestamp of last writeCommands()
    unsigned long lastThrottleTime  = 0; // timestamp of last writeThrottle()
};

#endif // SERVO_MANAGER_H
