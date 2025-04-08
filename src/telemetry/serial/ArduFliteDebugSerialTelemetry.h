/**
 * ArduFliteDebugSerialTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#pragma once

#include <Arduino.h>
#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"

class ArduFliteDebugSerialTelemetry : public ArduFliteTelemetry {
    public:
        ArduFliteDebugSerialTelemetry(float frequencyHz = 1.0f);
    
        void begin() override;
        void publish(const TelemetryData& data) override;
    
    private:
        static void telemetryTask(void* pvParameters);
    
        float             intervalMs;
        TelemetryData     pendingData;
        SemaphoreHandle_t telemetryMutex;
    };
    
