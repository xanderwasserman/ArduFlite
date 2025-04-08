/**
 * ServoManager.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ServoManager.h"

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
                           bool dual)
    : wingDesign(design), pitchConfig(pitchCfg), yawConfig(yawCfg),
      leftAilConfig(leftAilCfg), rightAilConfig(rightAilCfg), dualAilerons(dual)
{
    // Attach elevator and rudder servos.
    pitchServo.attach(pitchConfig.pin, pitchConfig.minPulse, pitchConfig.maxPulse);
    yawServo.attach(yawConfig.pin, yawConfig.minPulse, yawConfig.maxPulse);

    if (!dualAilerons) {
        singleAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
    } else {
        leftAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
        rightAilServo.attach(rightAilConfig.pin, rightAilConfig.minPulse, rightAilConfig.maxPulse);
    }
}

// Constructor for delta wing or V-tail designs.
ServoManager::ServoManager(WingDesign design,
                           ServoConfig surfaceLeftCfg,
                           ServoConfig surfaceRightCfg)
    : wingDesign(design), leftAilConfig(surfaceLeftCfg), rightAilConfig(surfaceRightCfg)
{
    // In these designs, we use the paired surfaces (e.g., elevons or ruddervators).
    dualAilerons = true;
    leftAilServo.attach(leftAilConfig.pin, leftAilConfig.minPulse, leftAilConfig.maxPulse);
    rightAilServo.attach(rightAilConfig.pin, rightAilConfig.minPulse, rightAilConfig.maxPulse);
}

void ServoManager::writeCommands(float rollCmd, float pitchCmd, float yawCmd) {
    // Depending on the wing design, mix commands differently.
    switch (wingDesign) {
        case CONVENTIONAL: {
            // Elevator (pitch control).
            int pitchDeflect = mapFloatToInt(pitchCmd, -1.0f, 1.0f,
                -pitchConfig.deflection, pitchConfig.deflection);
            if (pitchConfig.invert) {
                pitchDeflect = -pitchDeflect;
            }
            int pitchAngle = pitchConfig.neutral + pitchDeflect;
            pitchServo.write(pitchAngle);

            // Rudder (yaw control).
            int yawDeflect = mapFloatToInt(yawCmd, -1.0f, 1.0f,
                -yawConfig.deflection, yawConfig.deflection);
            if (yawConfig.invert) {
                yawDeflect = -yawDeflect;
            }
            int yawAngle = yawConfig.neutral + yawDeflect;
            yawServo.write(yawAngle);

            // Aileron(s) (roll control).
            int rollDeflect = mapFloatToInt(rollCmd, -1.0f, 1.0f,
                -leftAilConfig.deflection, leftAilConfig.deflection);
            // For conventional, you might want one servo to move opposite to the other.
            if (dualAilerons) {
                int leftAngle = leftAilConfig.neutral + (leftAilConfig.invert ? -rollDeflect : rollDeflect);
                int rightAngle = rightAilConfig.neutral + (rightAilConfig.invert ? rollDeflect : -rollDeflect);
                leftAilServo.write(leftAngle);
                rightAilServo.write(rightAngle);
            } else {
                int ailAngle = leftAilConfig.neutral + (leftAilConfig.invert ? -rollDeflect : rollDeflect);
                singleAilServo.write(ailAngle);
            }
            break;
        }
        case DELTA_WING: {
            // For delta wings, the elevons mix pitch and roll.
            // Typical mixing: leftElevon = pitch - roll, rightElevon = pitch + roll.
            int leftMix = mapFloatToInt((pitchCmd - rollCmd), -2.0f, 2.0f,
                -leftAilConfig.deflection, leftAilConfig.deflection);
            int rightMix = mapFloatToInt((pitchCmd + rollCmd), -2.0f, 2.0f,
                -rightAilConfig.deflection, rightAilConfig.deflection);
            if (leftAilConfig.invert) {
                leftMix = -leftMix;
            }
            if (rightAilConfig.invert) {
                rightMix = -rightMix;
            }
            int leftElevon = leftAilConfig.neutral + leftMix;
            int rightElevon = rightAilConfig.neutral + rightMix;
            leftAilServo.write(leftElevon);
            rightAilServo.write(rightElevon);
            break;
        }
        case V_TAIL: {
            // For V-tail, the ruddervators mix pitch and yaw.
            // Typical mixing: left = pitch + yaw, right = pitch - yaw.
            int leftMix = mapFloatToInt((pitchCmd + yawCmd), -2.0f, 2.0f,
                -leftAilConfig.deflection, leftAilConfig.deflection);
            int rightMix = mapFloatToInt((pitchCmd - yawCmd), -2.0f, 2.0f,
                -rightAilConfig.deflection, rightAilConfig.deflection);
            if (leftAilConfig.invert) {
                leftMix = -leftMix;
            }
            if (rightAilConfig.invert) {
                rightMix = -rightMix;
            }
            int leftRuddervator = leftAilConfig.neutral + leftMix;
            int rightRuddervator = rightAilConfig.neutral + rightMix;
            leftAilServo.write(leftRuddervator);
            rightAilServo.write(rightRuddervator);
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
