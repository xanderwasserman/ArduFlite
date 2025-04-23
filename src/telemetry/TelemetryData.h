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
    Vector3 accel, gyro;
    FliteQuaternion quat;
    EulerAngles orientation;
    EulerAngles attitudeSetpoint;
    EulerAngles rateSetpoint;
    EulerAngles attitudeCmd;
    EulerAngles rateCmd;
    float altitude;
    int   flight_state;
    int   flight_mode;

    void update(const ArduFliteIMU &myIMU, const ArduFliteController &myController) 
    {
        // Update accelerometer data
        accel = myIMU.getAcceleration();

        // Update gyroscope data
        gyro = myIMU.getGyro();

        // Update quaternion data
        quat = myIMU.getQuaternion();

        // Update orientation data
        orientation = myIMU.getOrientation();

        // Update attitude setpoint data
        attitudeSetpoint = myController.getAttitudeSetpoint();

        // Update rate setpoint data
        rateSetpoint = myController.getRateSetpoint();

        // Update attitude command data
        attitudeCmd = myController.getAttitudeCmd();

        // Update rate command data
        rateCmd = myController.getRateCmd();

        flight_state = static_cast<int>(myIMU.getFlightState());
        flight_mode = static_cast<int>( myController.getMode());

        altitude = myIMU.getAltitude();
    }
};

#endif
