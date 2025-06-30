// src/telemetry/flash/ArduFliteFlashTelemetry.cpp
//
// ArduFlite - Advanced Flight Controller Framework
// Author: Alexander Wasserman | Version: 1.0 | 25 May 2025
//
// Licensed under the MIT License. See LICENSE file for details.
//

#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

#include <FS.h>
#include <LittleFS.h>

#define FLUSH_INTERVAL_MS  500
#define MAX_ROW_BUFFER     600

ArduFliteFlashTelemetry::ArduFliteFlashTelemetry(float frequencyHz)
  : _intervalMs(1000.0f / frequencyHz),
    _mutex(nullptr),
    _lastFlushMs(0),
    _isLogging(false)
{ 

}

ArduFliteFlashTelemetry::~ArduFliteFlashTelemetry() 
{
    if (_mutex) 
    {
        {
            SemaphoreLock lock(_mutex);
            if (_isLogging) 
            {
                _logFile.flush();
                _logFile.close();
            }
        }
        vSemaphoreDelete(_mutex);
    }
}

void ArduFliteFlashTelemetry::begin() 
{
    if (!LittleFS.begin()) 
    {
        LOG_ERR("LittleFS mount failed; formatting...");
        LittleFS.format();
        if (!LittleFS.begin()) 
        {
            LOG_ERR("LittleFS format failed!");
            return;
        }
    }

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) 
    {
        LOG_ERR("Failed to create telemetry mutex");
        return;
    }

    // Report filesystem size
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    LOG_INF("LittleFS TotalBytes: %u", (unsigned)total);
    LOG_INF("LittleFS UsedBytes:  %u\n\n", (unsigned)used);

    // Start background task
    xTaskCreate(
        telemetryTask,
        "FlashTelTask",
        8192,
        this,
        1,
        nullptr
    );
}

void ArduFliteFlashTelemetry::publish(const TelemetryData& telemData, const ConfigData& configData)
{
    if (!_mutex) return;

    {
        SemaphoreLock lock(_mutex);
        _pendingData = telemData;
    }
}

int ArduFliteFlashTelemetry::findNextFlightLogIndex() 
{
    int maxIdx = -1;
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) 
    {
        String name = file.name();
        const char* p = name.c_str();
        if (p[0] == '/') p++;  // skip leading slash
        int idx;
        if (sscanf(p, "log_%03d.csv", &idx) == 1) 
        {
            maxIdx = max(maxIdx, idx);
        }
        file = root.openNextFile();
    }
    root.close();
    return maxIdx + 1;
}

void ArduFliteFlashTelemetry::startLogging() 
{
    if (!_mutex) return;

    {
        SemaphoreLock lock(_mutex);
        if (!_isLogging) 
        {
            int idx = findNextFlightLogIndex();
            char fn[32];
            snprintf(fn, sizeof(fn), "/log_%03d.csv", idx);
            _currentFilename = fn;

            _logFile = LittleFS.open(_currentFilename, FILE_WRITE);
            if (_logFile) 
            {
                // Write CSV header + newline
                char header[MAX_ROW_BUFFER];
                formatCSVHeader(header, sizeof(header));
                _logFile.write((const uint8_t*)header, strlen(header));
                _logFile.flush();
                _lastFlushMs = millis();
                _isLogging = true;
                LOG_INF("Logging started: %s", fn);
            } 
            else 
            {
                LOG_ERR("Failed to open %s", fn);
            }
        }
    }
}

void ArduFliteFlashTelemetry::stopLogging() 
{
    if (!_mutex) return;

    {
        SemaphoreLock lock(_mutex);
        if (_isLogging) 
        {
            _logFile.flush();
            _logFile.close();
            _isLogging = false;
            LOG_INF("Logging stopped: %s", _currentFilename.c_str());
        }
    }
}

void ArduFliteFlashTelemetry::listLogs() 
{
    if (!_mutex) return;

    {
        SemaphoreLock lock(_mutex);
        LOG("Available logs:");
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while (file) 
        {
            String name = file.name();
            if (name.startsWith("/")) name = name.substring(1);
            if (name.startsWith("log_") && name.endsWith(".csv")) 
            {
                LOG("%s", name);
            }
            file = root.openNextFile();
        }
        root.close();
    }
}

void ArduFliteFlashTelemetry::dumpLog(int index) 
{
    if (!_mutex) return;

    {
        SemaphoreLock lock(_mutex);
        char fn[32];
        snprintf(fn, sizeof(fn), "/log_%03d.csv", index);

        File f = LittleFS.open(fn, FILE_READ);
        if (!f) 
        {
            LOG_ERR("Failed to open %s", fn);
        } 
        else 
        {
            // BEGIN marker + blank line
            LOG_N("\n--- BEGIN %s ---\n\n", fn);

            // Read and print line by line
            while (f.available()) 
            {
                String line = f.readStringUntil('\n');
                LOG("%s", line.c_str());  // ensures each CSV row is on its own line
            }

            // END marker
            LOG_N("\n--- END %s ---\n", fn);
            f.close();
        }
    }
}

void ArduFliteFlashTelemetry::deleteLog(int index) 
{
    if (!_mutex) return;

    {
        SemaphoreLock lock(_mutex);
        char fn[32];
        snprintf(fn, sizeof(fn), "/log_%03d.csv", index);
        if (LittleFS.remove(fn)) 
        {
            LOG_INF("Deleted %s", fn);
        } 
        else 
        {
            LOG_ERR("Failed to delete %s", fn);
        }
    }
}

void ArduFliteFlashTelemetry::telemetryTask(void* pvParameters) 
{
    auto* self = static_cast<ArduFliteFlashTelemetry*>(pvParameters);
    char rowBuf[MAX_ROW_BUFFER];

    for (;;) 
    {
        unsigned long ts = millis();
        TelemetryData copy;

        // 1) Copy pending data under mutex
         if (self->_mutex) 
        {
            {
                SemaphoreLock lock(self->_mutex);
                copy = self->_pendingData;
            }
        }

        // 2) Format CSV row outside mutex
        size_t len = formatCSVRow(rowBuf, sizeof(rowBuf), ts, copy);

        // 3) Write & periodic flush under mutex
        if (self->_mutex) 
        {
           {
                SemaphoreLock lock(self->_mutex);
                if (self->_isLogging && self->_logFile) 
                {
                    self->_logFile.write((const uint8_t*)rowBuf, len);
                    if (ts - self->_lastFlushMs >= FLUSH_INTERVAL_MS) 
                    {
                        self->_logFile.flush();
                        self->_lastFlushMs = ts;
                    }
                }
            } 
        }
        

        // Maintain the desired telemetry rate
        unsigned long elapsed = millis() - ts;
        int delayMs = (int)max(1.0f, self->_intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

void ArduFliteFlashTelemetry::formatCSVHeader(char* buf, size_t bufSize) 
{
    const char* hdr =
        "timestamp,"
        "accel_x,accel_y,accel_z,"
        "gyro_x,gyro_y,gyro_z,"
        "quat_w,quat_x,quat_y,quat_z,"
        "roll,pitch,yaw,"
        "att_sp_roll,att_sp_pitch,att_sp_yaw,"
        "rate_sp_roll,rate_sp_pitch,rate_sp_yaw,"
        "att_cmd_roll,att_cmd_pitch,att_cmd_yaw,"
        "rate_cmd_roll,rate_cmd_pitch,rate_cmd_yaw,"
        "altitude,climb_rate,flight_state,flight_mode\n";
    // bufSize is at least MAX_ROW_BUFFER (600), plenty for this header
    strncpy(buf, hdr, bufSize - 1);
    buf[bufSize - 1] = '\0';
}

size_t ArduFliteFlashTelemetry::formatCSVRow(
    char* buf, size_t bufSize,
    unsigned long ts,
    const TelemetryData& d
) 
{
    // We have exactly 29 fields (1 timestamp + 26 floats + 2 ints)
    return snprintf(buf, bufSize,
        "%lu,"                             //  1 timestamp
        "%.3f,%.3f,%.3f,"                  //  2-4 accel
        "%.3f,%.3f,%.3f,"                  //  5-7 gyro
        "%.4f,%.4f,%.4f,%.4f,"             //  8-11 quat
        "%.3f,%.3f,%.3f,"                  // 12-14 orientation
        "%.3f,%.3f,%.3f,"                  // 15-17 attitudeSetpoint
        "%.3f,%.3f,%.3f,"                  // 18-20 rateSetpoint
        "%.3f,%.3f,%.3f,"                  // 21-23 attitudeCmd
        "%.3f,%.3f,%.3f,"                  // 24-26 rateCmd
        "%.2f,"                            // 27 altitude
        "%.2f,"                            // 28 climb_rate
        "%d,"                              // 29 flight_state
        "%d\n",                            // 30 flight_mode
        ts,
        // accel
        d.accel.x, d.accel.y, d.accel.z,
        // gyro
        d.gyro.x, d.gyro.y, d.gyro.z,
        // quat
        d.quat.w, d.quat.x, d.quat.y, d.quat.z,
        // orientation
        d.orientation.roll, d.orientation.pitch, d.orientation.yaw,
        // attitudeSetpoint
        d.attitudeSetpoint.roll, d.attitudeSetpoint.pitch, d.attitudeSetpoint.yaw,
        // rateSetpoint
        d.rateSetpoint.roll, d.rateSetpoint.pitch, d.rateSetpoint.yaw,
        // attitudeCmd
        d.attitudeCmd.roll, d.attitudeCmd.pitch, d.attitudeCmd.yaw,
        // rateCmd
        d.rateCmd.roll, d.rateCmd.pitch, d.rateCmd.yaw,
        // altitude + ints
        d.altitude,
        d.climb_rate,
        d.flight_state,
        d.flight_mode
    );
}

