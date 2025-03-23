#include "ServoManager.h"

// Helper function to map a float value from [inMin, inMax] to [outMin, outMax].
int ServoManager::mapFloatToInt(float val, float inMin, float inMax, int outMin, int outMax) {
    if (val > inMax) val = inMax;
    if (val < inMin) val = inMin;
    return (int)((val - inMin) * (float)(outMax - outMin) / (inMax - inMin) + outMin);
}

ServoManager::ServoManager(int leftOrSingleAileronPin, int rightAileronPin, int pitchPin, int yawPin)
    : pitchPin(pitchPin), yawPin(yawPin),
      leftAilPin(leftOrSingleAileronPin), rightAilPin(rightAileronPin)
{
    // Always attach pitch and yaw servos.
    pitchServo.attach(pitchPin, 500, 2500);
    yawServo.attach(yawPin, 500, 2500);

    if (rightAileronPin < 0) {
        // Single aileron mode.
        dualAilerons = false;
        singleAilServo.attach(leftOrSingleAileronPin, 500, 2500);
    } else {
        // Dual aileron mode.
        dualAilerons = true;
        leftAilServo.attach(leftOrSingleAileronPin, 500, 2500);
        rightAilServo.attach(rightAileronPin, 500, 2500);
    }
}

void ServoManager::writeCommands(float rollCmd, float pitchCmd, float yawCmd) {
    // Map pitchCmd from [-1, +1] to a deflection range, then add 90 for servo mid-point.
    int pitchDeg = mapFloatToInt(pitchCmd, -1.0f, 1.0f, -70, 70);
    pitchDeg = 90 + pitchDeg;

    // Map yawCmd similarly.
    int yawDeg = mapFloatToInt(yawCmd, -1.0f, 1.0f, -70, 70);
    yawDeg = 90 - yawDeg;

    pitchServo.write(pitchDeg);
    yawServo.write(yawDeg);

    if (!dualAilerons) {
        // Single aileron mode.
        int ailDeflect = mapFloatToInt(rollCmd, -1.0f, 1.0f, -70, 70);
        int ailAngle = 90 - ailDeflect;
        singleAilServo.write(ailAngle);
    } else {
        // Dual aileron mode.
        int ailDeflect = mapFloatToInt(rollCmd, -1.0f, 1.0f, -70, 70);
        int leftAngle  = 90 - ailDeflect;
        int rightAngle = 90 - ailDeflect;
        leftAilServo.write(leftAngle);
        rightAilServo.write(rightAngle);
    }
}
