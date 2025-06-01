/**
 * ArduFliteDebugSerialTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_DEBUG_SERIAL_TELEMETRY_H
#define ARDUFLITE_DEBUG_SERIAL_TELEMETRY_H

#include <Arduino.h>
#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"

class ArduFliteDebugSerialTelemetry : public ArduFliteTelemetry {
    public:
        ArduFliteDebugSerialTelemetry(float frequencyHz = 1.0f);
    
        void begin() override;
        void publish(const TelemetryData& telemData, const ConfigData& configData)  override;
    
    private:
        static void telemetryTask(void* pvParameters);
    
        float             intervalMs;
        TelemetryData     pendingData;
        SemaphoreHandle_t telemetryMutex;
    };
    
#endif //ARDUFLITE_DEBUG_SERIAL_TELEMETRY_H