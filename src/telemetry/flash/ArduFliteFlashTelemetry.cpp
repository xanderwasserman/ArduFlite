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

namespace FlashTelemetryConfig {
    constexpr unsigned long FLUSH_INTERVAL_MS = 500;   ///< Flush to flash every 500ms
    constexpr size_t MAX_ROW_BUFFER = 600;             ///< Max CSV row size in bytes
}

ArduFliteFlashTelemetry::ArduFliteFlashTelemetry(float frequencyHz)
  : _intervalMs(1000.0f / frequencyHz),
    _dataMutex(nullptr),
    _fileMutex(nullptr),
    _lastFlushMs(0),
    _isLogging(false)
{
}

ArduFliteFlashTelemetry::~ArduFliteFlashTelemetry() 
{
    if (_fileMutex) 
    {
        {
            SemaphoreLock lock(_fileMutex);
            if (_isLogging) 
            {
                _logFile.flush();
                _logFile.close();
            }
        }
        vSemaphoreDelete(_fileMutex);
    }
    if (_dataMutex)
    {
        vSemaphoreDelete(_dataMutex);
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

    _dataMutex = xSemaphoreCreateMutex();
    _fileMutex = xSemaphoreCreateMutex();
    if (!_dataMutex || !_fileMutex) 
    {
        LOG_ERR("Failed to create flash telemetry mutexes");
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
    if (!_dataMutex) return;

    // Fast operation - only copy data, don't wait for file I/O
    // Use short timeout - if we miss one sample, next one will succeed
    SemaphoreLock lock(_dataMutex);
    if (!lock.acquired()) return;  // Skip this sample rather than block
    _pendingData = telemData;
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
    using namespace FlashTelemetryConfig;
    if (!_fileMutex) return;

    SemaphoreLock lock(_fileMutex, portMAX_DELAY);  // File ops can wait
    if (!lock.acquired()) return;
    
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

void ArduFliteFlashTelemetry::stopLogging() 
{
    if (!_fileMutex) return;

    SemaphoreLock lock(_fileMutex, portMAX_DELAY);
    if (!lock.acquired()) return;
    
    if (_isLogging) 
    {
        _logFile.flush();
        _logFile.close();
        _isLogging = false;
        LOG_INF("Logging stopped: %s", _currentFilename.c_str());
    }
}

void ArduFliteFlashTelemetry::listLogs() 
{
    if (!_fileMutex) return;

    SemaphoreLock lock(_fileMutex, portMAX_DELAY);
    if (!lock.acquired()) return;
    
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

void ArduFliteFlashTelemetry::dumpLog(int index) 
{
    if (!_fileMutex) return;

    SemaphoreLock lock(_fileMutex, portMAX_DELAY);
    if (!lock.acquired()) return;
    
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

void ArduFliteFlashTelemetry::deleteLog(int index) 
{
    if (!_fileMutex) return;

    SemaphoreLock lock(_fileMutex, portMAX_DELAY);
    if (!lock.acquired()) return;
    
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

void ArduFliteFlashTelemetry::telemetryTask(void* pvParameters) 
{
    using namespace FlashTelemetryConfig;
    auto* self = static_cast<ArduFliteFlashTelemetry*>(pvParameters);
    char rowBuf[MAX_ROW_BUFFER];

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(self->_intervalMs);

    for (;;) 
    {
        unsigned long ts = millis();

        // 1) FAST: Copy pending data under data mutex (sub-millisecond)
        if (self->_dataMutex) 
        {
            SemaphoreLock lock(self->_dataMutex);
            if (lock.acquired())
            {
                self->_writeBuffer = self->_pendingData;
            }
            // If lock failed, use previous _writeBuffer (stale but safe)
        }

        // 2) Format CSV row outside any mutex
        size_t len = formatCSVRow(rowBuf, sizeof(rowBuf), ts, self->_writeBuffer);

        // 3) SLOW: Write to flash under file mutex (doesn't block publish)
        if (self->_fileMutex) 
        {
            SemaphoreLock lock(self->_fileMutex);
            if (lock.acquired() && self->_isLogging && self->_logFile) 
            {
                size_t written = self->_logFile.write((const uint8_t*)rowBuf, len);
                if (written < len)
                {
                    LOG_ERR("Flash write failed: %u/%u bytes", (unsigned)written, (unsigned)len);
                }
                if (ts - self->_lastFlushMs >= FLUSH_INTERVAL_MS) 
                {
                    self->_logFile.flush();
                    self->_lastFlushMs = ts;
                }
            }
        }

        // Maintain consistent timing with vTaskDelayUntil
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void ArduFliteFlashTelemetry::formatCSVHeader(char* buf, size_t bufSize) 
{
    using namespace FlashTelemetryConfig;
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
    // bufSize should be at least MAX_ROW_BUFFER (600)
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

