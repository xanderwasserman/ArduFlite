#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>
#include "ArduFliteIMU.h"

struct TelemetryData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float qw, qx, qy, qz;
    float pitch, roll, yaw;
    float rollCmd, pitchCmd, yawCmd;

    void update(const ArduFliteIMU &myIMU, float roll_command, float pitch_command, float yaw_command) {
        // Update accelerometer data
        accelX = myIMU.getAccelX();
        accelY = myIMU.getAccelY();
        accelZ = myIMU.getAccelZ();

        // Update gyroscope data
        gyroX = myIMU.getGyroX();
        gyroY = myIMU.getGyroY();
        gyroZ = myIMU.getGyroZ();

        // Update quaternion data
        qw = myIMU.getQw();
        qx = myIMU.getQx();
        qy = myIMU.getQy();
        qz = myIMU.getQz();

        // Update orientation data
        pitch = myIMU.getPitch();
        roll  = myIMU.getRoll();
        yaw   = myIMU.getYaw();

        // Update command data
        rollCmd  = roll_command;
        pitchCmd = pitch_command;
        yawCmd   = yaw_command;
    }
};

#endif
