// ServoManager.h
#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <Arduino.h>
#include <ESP32Servo.h> // for ESP32 boards; adjust if needed

static int mapFloatToInt(float val, float inMin, float inMax, int outMin, int outMax) {
    if (val > inMax) val = inMax;
    if (val < inMin) val = inMin;
    return (int)((val - inMin) * (float)(outMax - outMin) / (inMax - inMin) + outMin);
}

class ServoManager {
public:
    // Pass -1 (or any negative) for 'rightAileronPin' if you only have one aileron servo.
    // If 'rightAileronPin' >= 0, we'll use dual-aileron logic.
    ServoManager(int leftOrSingleAileronPin, int rightAileronPin,
                 int pitchPin, int yawPin)
      : pitchPin(pitchPin), yawPin(yawPin), 
        leftAilPin(leftOrSingleAileronPin), rightAilPin(rightAileronPin)
    {
        // Attach pitch & yaw always
        pitchServo.attach(pitchPin,  500, 2500);
        yawServo.attach(yawPin,      500, 2500);

        if (rightAileronPin < 0) {
            // SINGLE AILERON MODE
            dualAilerons = false;
            singleAilServo.attach(leftOrSingleAileronPin, 500, 2500);
        } else {
            // DUAL AILERON MODE
            dualAilerons = true;
            leftAilServo.attach(leftOrSingleAileronPin,  500, 2500);
            rightAilServo.attach(rightAileronPin,        500, 2500);
        }
    }

    // Accepts rollCmd, pitchCmd, yawCmd in [-1..+1]
    void writeCommands(float rollCmd, float pitchCmd, float yawCmd) {
        // Pitch & yaw: just map to [0..180] for a typical servo
        int pitchDeg = mapFloatToInt(pitchCmd, -1.0f, 1.0f, -45, 45);
        pitchDeg   = 90 + pitchDeg;

        int yawDeg   = mapFloatToInt(yawCmd,   -1.0f, 1.0f, -45, 45);
        yawDeg   = 90 + yawDeg;

        pitchServo.write(pitchDeg);
        yawServo.write(yawDeg);

        if (!dualAilerons) {
            // SINGLE aileron servo
            int ailDeflect = mapFloatToInt(rollCmd, -1.0f, 1.0f, -70, 70);

            int ailAngle   = 90 + ailDeflect;
            singleAilServo.write(ailAngle);
        } else {
            // DUAL ailerons
            // Example: left up when right down
            int ailDeflect = mapFloatToInt(rollCmd, -1.0f, 1.0f, -70, 70);

            int leftAngle  = 90 + ailDeflect;
            int rightAngle = 90 + ailDeflect;
            leftAilServo.write(leftAngle);
            rightAilServo.write(rightAngle);
        }
    }

private:
    // Pin info stored, if you ever need it
    int pitchPin, yawPin;
    int leftAilPin, rightAilPin;

    // Are we using single or dual ailerons?
    bool dualAilerons;

    // Servos
    Servo pitchServo, yawServo;
    // Single aileron servo OR left & right aileron servos
    Servo singleAilServo, leftAilServo, rightAilServo;
};

#endif // SERVO_MANAGER_H
