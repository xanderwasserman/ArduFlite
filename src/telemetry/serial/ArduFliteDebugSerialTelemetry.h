/**
 * ArduFliteDebugSerialTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_DEBUG_SERIAL_TELEMETRY_H
#define ARDUFLITE_DEBUG_SERIAL_TELEMETRY_H

#include "src/telemetry/PeriodicTelemetryBackend.h"

/**
 * @brief Full-data debug serial telemetry backend.
 *
 * Publishes all TelemetryData fields to Serial at a configurable rate from a
 * dedicated FreeRTOS task. Intended for ground-connected development sessions;
 * not for in-flight use where a Serial connection is unavailable.
 */
class ArduFliteDebugSerialTelemetry final : public PeriodicTelemetryBackend {
    public:
        explicit ArduFliteDebugSerialTelemetry(float frequencyHz = 1.0f);

    private:
        void runLoop() override;
    };

#endif //ARDUFLITE_DEBUG_SERIAL_TELEMETRY_H
