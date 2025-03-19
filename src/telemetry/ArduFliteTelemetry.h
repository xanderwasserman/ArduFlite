// ArduFliteTelemetry.h
#pragma once

#include "TelemetryData.h"

class ArduFliteTelemetry {
public:
    virtual ~ArduFliteTelemetry() {}

    // Called once to initialize, connect, etc.
    virtual void begin() = 0;

    // Called in loop() or from a separate thread, etc.
    virtual void publish(const TelemetryData& data) = 0;

    // Allows resetting or reconfiguration
    // (default is empty if a derived class doesn't need it)
    virtual void reset() {}
};
