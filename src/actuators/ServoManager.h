#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoManager {
public:
    /**
     * Constructor.
     * @param leftOrSingleAileronPin Digital pin for the left (or single) aileron servo.
     * @param rightAileronPin Digital pin for the right aileron servo.
     *                        Pass a negative value (e.g. -1) if using a single aileron.
     * @param pitchPin Digital pin for the pitch servo.
     * @param yawPin Digital pin for the yaw servo.
     */
    ServoManager(int leftOrSingleAileronPin, int rightAileronPin,
                 int pitchPin, int yawPin);

    /**
     * Write control commands to the servos.
     * @param rollCmd  Command for roll in the range [-1, +1]
     * @param pitchCmd Command for pitch in the range [-1, +1]
     * @param yawCmd   Command for yaw in the range [-1, +1]
     */
    void writeCommands(float rollCmd, float pitchCmd, float yawCmd);

private:
    // Helper: maps a float value from one range to another.
    static int mapFloatToInt(float val, float inMin, float inMax, int outMin, int outMax);

    // Pin numbers
    int pitchPin, yawPin;
    int leftAilPin, rightAilPin;

    // Mode: true if using dual aileron servos, false if using a single servo.
    bool dualAilerons;

    // Servo objects for pitch, yaw, and ailerons.
    Servo pitchServo, yawServo;
    Servo singleAilServo, leftAilServo, rightAilServo;
};

#endif // SERVO_MANAGER_H
