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
        // Update data from ArduFlite IMU
        accel = myIMU.getAcceleration();
        gyro = myIMU.getGyro();
        quat = myIMU.getQuaternion();
        orientation = myIMU.getOrientation();

        // Update data from ArduFlite Controller
        attitudeSetpoint = myController.getAttitudeSetpoint();
        rateSetpoint = myController.getRateSetpoint();
        attitudeCmd = myController.getAttitudeCmd();
        rateCmd = myController.getRateCmd();

        // Update flight state and mode
        flight_state = static_cast<int>(myIMU.getFlightState());
        flight_mode = static_cast<int>( myController.getMode());

        // Update additional flight data
        altitude = myIMU.getAltitude();
    }
};

#endif
