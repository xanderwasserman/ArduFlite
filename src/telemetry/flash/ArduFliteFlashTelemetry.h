/**
 * ArduFliteFlashTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#ifndef ARDUFLITE_FLASH_TELEMETRY_H
#define ARDUFLITE_FLASH_TELEMETRY_H

#include "src/telemetry/ArduFliteTelemetry.h"

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

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

#endif //ARDUFLITE_FLASH_TELEMETRY_H