// ArduFliteFlashTelemetry.h

#pragma once

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "src/telemetry/ArduFliteTelemetry.h"

class ArduFliteFlashTelemetry : public ArduFliteTelemetry {
public:
    explicit ArduFliteFlashTelemetry(float frequencyHz = 50.0f);
    ~ArduFliteFlashTelemetry();

    void begin() override;
    void publish(const TelemetryData& data) override;

    // Call on launch/landing
    void startLogging();
    void stopLogging();

    // File management
    void listLogs();
    void dumpLog(int index);
    void deleteLog(int index);

    // Erase entire filesystem (use with care)
    void reset() override { 
        if (_mutex) {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            LittleFS.format();
            xSemaphoreGive(_mutex);
        }
    }

private:
    static void telemetryTask(void* pvParameters);

    // Helpers (all called under mutex)
    int  findNextFlightLogIndex();
    static void formatCSVHeader(char* buf, size_t bufSize);
    static size_t formatCSVRow(char* buf, size_t bufSize, unsigned long ts, const TelemetryData& d);

    float             _intervalMs;
    SemaphoreHandle_t _mutex;
    TelemetryData     _pendingData;
    unsigned long     _lastFlushMs;
    File              _logFile;
    bool              _isLogging;
    String            _currentFilename;
};
