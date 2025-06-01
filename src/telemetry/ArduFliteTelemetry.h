/**
 * ArduFliteTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_TELEMETRY_H
#define ARDUFLITE_TELEMETRY_H

#include "src/telemetry/TelemetryData.h"
#include "src/telemetry/ConfigData.h"

class ArduFliteTelemetry {
public:
    virtual ~ArduFliteTelemetry() {}

    // Called once to initialize, connect, etc.
    virtual void begin() = 0;

    // Called in loop() or from a separate thread, etc.
    virtual void publish(const TelemetryData& telemData, const ConfigData& configData) = 0;

    // Allows resetting or reconfiguration
    // (default is empty if a derived class doesn't need it)
    virtual void reset() {}
};

#endif //ARDUFLITE_TELEMETRY_H