/**
 * ArduFliteQSerialTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_Q_SERIAL_TELEMETRY_H
#define ARDUFLITE_Q_SERIAL_TELEMETRY_H

#include "src/telemetry/PeriodicTelemetryBackend.h"
#include "src/telemetry/TelemetryData.h"

/**
 * @brief Quaternion-only serial telemetry backend ("Q" = quaternion).
 *
 * Streams the attitude quaternion (w, x, y, z) as a compact CSV line at a
 * configurable rate over Serial. Designed for consumption by real-time 3-D
 * visualisation tools (e.g. tools/visualisation/ in this repository).
 *
 * For full-fidelity CSV flight logging to on-board flash, use
 * ArduFliteFlashTelemetry instead.
 */
class ArduFliteQSerialTelemetry final : public PeriodicTelemetryBackend {
    public:
        explicit ArduFliteQSerialTelemetry(float frequencyHz = 10.0f);

    private:
        void runLoop() override;
    };

#endif //ARDUFLITE_Q_SERIAL_TELEMETRY_H
