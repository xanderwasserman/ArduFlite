#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>

struct TelemetryData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float qw, qx, qy, qz;
    float pitch, roll, yaw;
    float rollCmd, pitchCmd, yawCmd;
};

#endif
