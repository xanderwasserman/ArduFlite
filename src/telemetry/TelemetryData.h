/**
 * TelemetryData.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>
#include "src/orientation/ArduFliteIMU.h"

struct TelemetryData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float qw, qx, qy, qz;
    float pitch, roll, yaw;
    float rollRateCmd, pitchRateCmd, yawRateCmd;
    float rollCmd, pitchCmd, yawCmd;
    int   flight_state;
    float altitude;

    void update(const ArduFliteIMU &myIMU, float rollRate_command, float pitchRate_command, float yawRate_command, float roll_command, float pitch_command, float yaw_command, FlightState state) {
        // Update accelerometer data
        Vector3 accel = myIMU.getAcceleration();
        accelX = accel.x;
        accelY = accel.y;
        accelZ = accel.z;

        // Update gyroscope data
        Vector3 gyro = myIMU.getGyro();
        gyroX = gyro.x;
        gyroY = gyro.y;
        gyroZ = gyro.z;

        // Update quaternion data
        FliteQuaternion q = myIMU.getQuaternion();
        qw = q.w;
        qx = q.x;
        qy = q.y;
        qz = q.z;

        // Update orientation data
        EulerAngles orientation = myIMU.getOrientation();
        pitch = orientation.pitch;
        roll  = orientation.roll;
        yaw   = orientation.yaw;

        // Update command data
        rollRateCmd  = rollRate_command;
        pitchRateCmd = pitchRate_command;
        yawRateCmd   = yawRate_command;

        rollCmd  = roll_command;
        pitchCmd = pitch_command;
        yawCmd   = yaw_command;

        flight_state = static_cast<int>(state);

        altitude = myIMU.getAltitude();
    }
};

#endif
