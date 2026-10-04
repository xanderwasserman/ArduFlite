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

#include <chrono>

#include "src/telemetry/TelemetryData.h"

/**
 * @brief The short lock timeout every telemetry backend uses.
 *
 * Deliberately a WAIT and not a try-lock: publish() and the writer task both
 * run at 50 Hz, so brief overlap is normal, and giving up instantly would drop
 * samples that a 5 ms wait comfortably catches.
 *
 * Named here rather than repeated, because it is one decision shared by four
 * backends.
 */
inline constexpr std::chrono::milliseconds kTelemetryLockTimeout{ 5 };

class ArduFliteTelemetry {
public:
    virtual ~ArduFliteTelemetry() {}

    // Called once to initialize, connect, etc.
    virtual void begin() = 0;

    // Called in loop() or from a separate thread, etc.
    virtual void publish(const TelemetryData& telemData) = 0;

    // Allows resetting or reconfiguration
    // (default is empty if a derived class doesn't need it)
    virtual void reset() {}
};

#endif //ARDUFLITE_TELEMETRY_H