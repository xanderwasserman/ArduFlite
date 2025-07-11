/**
 * ServoManager.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ServoManager.h"
#include "include/ServoConfiguration.h"
#include "src/utils/Logging.h"

// Helper: maps a float value from [inMin, inMax] to [outMin, outMax].
int ServoManager::mapFloatToInt(float val, float inMin, float inMax, int outMin, int outMax) {
    if (val > inMax) val = inMax;
    if (val < inMin) val = inMin;
    return (int)((val - inMin) * (float)(outMax - outMin) / (inMax - inMin) + outMin);
}

// Constructor for conventional wing designs.
ServoManager::ServoManager(WingDesign design,
                           ServoConfig pitchCfg,
                           ServoConfig yawCfg,
                           ServoConfig leftAilCfg,
                           ServoConfig rightAilCfg,
                           ServoConfig throttleCfg,
                           bool dual)
    : wingDesign(design), pitchConfig(pitchCfg), yawConfig(yawCfg),
      leftAilConfig(leftAilCfg), rightAilConfig(rightAilCfg), dualAilerons(dual),
      throttleConfig(throttleCfg), maxServoDegPerSec(ServoSetupConfig::MAX_SERVO_DEG_PER_SEC),
      maxThrottlePerSec(ServoSetupConfig::MAX_THROTTLE_PER_SEC)
{
    // Attach elevator and rudder servos.
    pitchServo.attach(pitchConfig.pin, pitchConfig.minPulse, pitchConfig.maxPulse);
    yawServo.attach(yawConfig.pin, yawConfig.minPulse, yawConfig.maxPulse);
    throttleServo.attach(throttleConfig.pin, throttleConfig.minPulse, throttleConfig.maxPulse);

    if (!dualAilerons) {
        singleAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
    } else {
        leftAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
        rightAilServo.attach(rightAilConfig.pin, rightAilConfig.minPulse, rightAilConfig.maxPulse);
    }

    lastThrottleTime = micros();
    lastThrottleCmd = 0.0f;

    lastUpdateMicros   = micros();
    lastPitchAngleDeg  = pitchConfig.neutral;
    lastYawAngleDeg    = yawConfig.neutral;
    if (dualAilerons) {
        lastLeftAngleDeg  = leftAilConfig.neutral;
        lastRightAngleDeg = rightAilConfig.neutral;
    } else {
        lastSingleAilDeg  = leftAilConfig.neutral;
    }
}

// Constructor for delta wing or V-tail designs.
ServoManager::ServoManager(WingDesign design,
                           ServoConfig surfaceLeftCfg,
                           ServoConfig surfaceRightCfg,
                           ServoConfig throttleCfg)
    : wingDesign(design), leftAilConfig(surfaceLeftCfg), rightAilConfig(surfaceRightCfg),
    throttleConfig(throttleCfg)
{
    // In these designs, we use the paired surfaces (e.g., elevons or ruddervators).
    dualAilerons = true;
    leftAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
    rightAilServo.attach(rightAilConfig.pin, rightAilConfig.minPulse, rightAilConfig.maxPulse);
    throttleServo.attach(throttleConfig.pin, throttleConfig.minPulse, throttleConfig.maxPulse);

    lastThrottleTime = micros();
    lastThrottleCmd = 0.0f;

    lastUpdateMicros  = micros();
    // only initialize the aileron/elevon state:
    lastLeftAngleDeg  = leftAilConfig.neutral;
    lastRightAngleDeg = rightAilConfig.neutral;
}

void ServoManager::writeThrottle(float throttleCmd)
{
    // clamp input [0…1]
    throttleCmd = constrain(throttleCmd, 0.0f, 1.0f);
    if (throttleCmd < 0.05f) throttleCmd = 0.0f;

    // compute elapsed time
    unsigned long now = micros();
    float dt = (now - lastThrottleTime) * 1e-6f;
    lastThrottleTime = now;

    // limit how fast we can change throttle
    float maxDelta = maxThrottlePerSec * dt;
    throttleCmd = constrain(throttleCmd, lastThrottleCmd - maxDelta, lastThrottleCmd + maxDelta);
    lastThrottleCmd = throttleCmd;

    // map into ESC pulse [minPulse…maxPulse]
    int pulse = throttleConfig.minPulse + (int)(throttleCmd * float(throttleConfig.maxPulse - throttleConfig.minPulse));
    throttleServo.writeMicroseconds(pulse);
}

void ServoManager::writeCommands(float rollCmd, float pitchCmd, float yawCmd) 
{
    // --- Compute loop dt (seconds) ---
    unsigned long now = micros();
    float dt = (now - lastUpdateMicros) * 1e-6f;
    lastUpdateMicros = now;

    // Maximum allowed change this cycle (degrees)
    float maxDelta = maxServoDegPerSec * dt;

    switch (wingDesign) 
    {
        case CONVENTIONAL: 
        {
            // 1) Elevator (pitch control)
            int rawPitch = mapFloatToInt(pitchCmd, -1.0f, 1.0f, -pitchConfig.deflection, pitchConfig.deflection);
            if (pitchConfig.invert) rawPitch = -rawPitch;
            float desiredPitchAngle = pitchConfig.neutral + rawPitch;
            // Slew‑limit
            float limitedPitch = constrain(desiredPitchAngle, lastPitchAngleDeg - maxDelta, lastPitchAngleDeg + maxDelta);
            lastPitchAngleDeg = limitedPitch;
            pitchServo.write((int)limitedPitch);

            // 2) Rudder (yaw control)
            int rawYaw = mapFloatToInt(yawCmd, -1.0f, 1.0f, -yawConfig.deflection, yawConfig.deflection);
            if (yawConfig.invert) rawYaw = -rawYaw;
            float desiredYawAngle = yawConfig.neutral + rawYaw;
            float limitedYaw = constrain(desiredYawAngle, lastYawAngleDeg - maxDelta, lastYawAngleDeg + maxDelta);
            lastYawAngleDeg = limitedYaw;
            yawServo.write((int)limitedYaw);

            // 3) Ailerons (roll control)
            int rawRollDeflect = mapFloatToInt(rollCmd, -1.0f, 1.0f, -leftAilConfig.deflection, leftAilConfig.deflection);
            if (dualAilerons) 
            {
                float desiredLeft  = leftAilConfig.neutral + (leftAilConfig.invert ? -rawRollDeflect : rawRollDeflect);
                float desiredRight = rightAilConfig.neutral + (rightAilConfig.invert ? rawRollDeflect : -rawRollDeflect);

                float limitedLeft  = constrain(desiredLeft, lastLeftAngleDeg  - maxDelta, lastLeftAngleDeg  + maxDelta);
                float limitedRight = constrain(desiredRight, lastRightAngleDeg - maxDelta, lastRightAngleDeg + maxDelta);

                lastLeftAngleDeg  = limitedLeft;
                lastRightAngleDeg = limitedRight;
                leftAilServo.write((int)limitedLeft);
                rightAilServo.write((int)limitedRight);
            }
            else 
            {
                float desiredAil = leftAilConfig.neutral + (leftAilConfig.invert ? -rawRollDeflect : rawRollDeflect);
                float limitedAil = constrain(desiredAil, lastSingleAilDeg - maxDelta, lastSingleAilDeg + maxDelta);
                lastSingleAilDeg = limitedAil; 
                singleAilServo.write((int)limitedAil);
            }
            break;
        }

        case DELTA_WING: 
        {
            // Elevon mixing: left = pitch – roll, right = pitch + roll
            int rawLeftMix  = mapFloatToInt(pitchCmd - rollCmd, -2.0f, 2.0f, -leftAilConfig.deflection, leftAilConfig.deflection);
            int rawRightMix = mapFloatToInt(pitchCmd + rollCmd, -2.0f, 2.0f, -rightAilConfig.deflection, rightAilConfig.deflection);
            if (leftAilConfig.invert)  rawLeftMix  = -rawLeftMix;
            if (rightAilConfig.invert) rawRightMix = -rawRightMix;

            float desiredLeft  = leftAilConfig.neutral  + rawLeftMix;
            float desiredRight = rightAilConfig.neutral + rawRightMix;

            float limitedLeft  = constrain(desiredLeft, lastLeftAngleDeg  - maxDelta, lastLeftAngleDeg  + maxDelta);
            float limitedRight = constrain(desiredRight, lastRightAngleDeg - maxDelta, lastRightAngleDeg + maxDelta);

            lastLeftAngleDeg  = limitedLeft;
            lastRightAngleDeg = limitedRight;
            leftAilServo.write((int)limitedLeft);
            rightAilServo.write((int)limitedRight);
            break;
        }
        case V_TAIL: 
        {
            // Ruddervator mixing: left = pitch + yaw, right = pitch – yaw
            int rawLeftMix  = mapFloatToInt(pitchCmd + yawCmd, -2.0f, 2.0f, -leftAilConfig.deflection, leftAilConfig.deflection);
            int rawRightMix = mapFloatToInt(pitchCmd - yawCmd, -2.0f, 2.0f, -rightAilConfig.deflection, rightAilConfig.deflection);
            if (leftAilConfig.invert)  rawLeftMix  = -rawLeftMix;
            if (rightAilConfig.invert) rawRightMix = -rawRightMix;

            float desiredLeft  = leftAilConfig.neutral  + rawLeftMix;
            float desiredRight = rightAilConfig.neutral + rawRightMix;

            float limitedLeft  = constrain(desiredLeft, lastLeftAngleDeg  - maxDelta, lastLeftAngleDeg  + maxDelta);
            float limitedRight = constrain(desiredRight, lastRightAngleDeg - maxDelta, lastRightAngleDeg + maxDelta);

            lastLeftAngleDeg  = limitedLeft;
            lastRightAngleDeg = limitedRight;
            leftAilServo.write((int)limitedLeft);
            rightAilServo.write((int)limitedRight);
            break;
        }
    }
}

// --- Setter methods for inversion flags ---

void ServoManager::setPitchInversion(bool invert) {
    pitchConfig.invert = invert;
}

void ServoManager::setYawInversion(bool invert) {
    yawConfig.invert = invert;
}

void ServoManager::setLeftSurfaceInversion(bool invert) {
    leftAilConfig.invert = invert;
}

void ServoManager::setRightSurfaceInversion(bool invert) {
    rightAilConfig.invert = invert;
}

// --- Optional setters to update configurations at runtime ---

void ServoManager::setPitchConfig(const ServoConfig &config) {
    pitchConfig = config;
    pitchServo.detach();
    pitchServo.attach(pitchConfig.pin, pitchConfig.minPulse, pitchConfig.maxPulse);
}

void ServoManager::setYawConfig(const ServoConfig &config) {
    yawConfig = config;
    yawServo.detach();
    yawServo.attach(yawConfig.pin, yawConfig.minPulse, yawConfig.maxPulse);
}

void ServoManager::setLeftSurfaceConfig(const ServoConfig &config) {
    leftAilConfig = config;
    if (dualAilerons)
    {
        leftAilServo.detach();
        leftAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
    }
    else
    {
        singleAilServo.detach();
        singleAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
    }
}

void ServoManager::setRightSurfaceConfig(const ServoConfig &config) {
    rightAilConfig = config;
    if (dualAilerons) {
        rightAilServo.detach();
        rightAilServo.attach(rightAilConfig.pin, rightAilConfig.minPulse, rightAilConfig.maxPulse);
    }
}

/**
 * @brief Runs a self-test routine by wiggling control surfaces.
 *
 * Moves the control surfaces to their full positive, full negative, and neutral positions
 * in sequence over multiple cycles, ensuring the servo outputs respond correctly.
 * This is useful for verifying hardware functionality during startup or in the field.
 */
void ServoManager::testControlSurfaces() 
{
    const int numCycles = 2;    // Number of test cycles
    const int delayTime = 500;    // Delay (in milliseconds) between commands

    LOG_INF("Beginning control surface test...");

    for (int i = 0; i < numCycles; i++) {
        // Command maximum deflection (normalized value 1.0)
        writeCommands(1.0f, 1.0f, 1.0f);
        delay(delayTime);

        // Command minimum deflection (normalized value -1.0)
        writeCommands(-1.0f, -1.0f, -1.0f);
        delay(delayTime);

        // Command neutral position (normalized value 0.0)
        writeCommands(0.0f, 0.0f, 0.0f);
        delay(delayTime);
    }
    LOG_INF("Control surface test complete.");
}
