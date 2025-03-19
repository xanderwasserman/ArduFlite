// ArduFliteQSerialTelemetry.h
#pragma once

#include <Arduino.h>
#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"

class ArduFliteQSerialTelemetry : public IArduFliteTelemetry {
    public:
        ArduFliteQSerialTelemetry(float frequencyHz = 10.0f);
    
        void begin() override;
        void publish(const TelemetryData& data) override;
    
    private:
        static void telemetryTask(void* pvParameters);
    
        float             intervalMs;
        TelemetryData     pendingData;
        SemaphoreHandle_t telemetryMutex;
    };
    
