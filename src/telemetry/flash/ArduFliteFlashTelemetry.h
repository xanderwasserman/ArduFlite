/**
 * ArduFliteFlashTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#ifndef ARDUFLITE_FLASH_TELEMETRY_H
#define ARDUFLITE_FLASH_TELEMETRY_H

#include "src/telemetry/ArduFliteTelemetry.h"
#include "include/ArduFlite.h"
#include "src/utils/Logging.h"

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <atomic>

class ArduFliteFlashTelemetry : public ArduFliteTelemetry
{
public:
    explicit ArduFliteFlashTelemetry(float frequencyHz = 10.0f);
    ~ArduFliteFlashTelemetry();

    void begin() override;
    void publish(const TelemetryData& telemData) override;

    // Call on launch/landing.
    /// @brief Begin recording a new flight log.
    /// @return true if a log is recording after the call (newly started, or already
    ///         active from a prior call); false on any failure (mutex timeout, flash
    ///         full with no purgeable logs, index space exhausted, or file-open error).
    ///         Callers on the launch/arm path should surface a false result — it means
    ///         the flight is NOT being recorded.
    /// @note startLogging() can hold _fileMutex for several seconds if the
    ///       auto-purge loop fires. Do not call from tasks at priority > 1.
    bool startLogging();
    bool stopLogging();

    /// @brief Returns true if a log file is currently open and recording.
    bool isLogging() const { return _isLogging; }

    // File management — ground-only diagnostic commands.
    /// @note listLogs() holds _fileMutex during serial output. Acceptable for ground-only
    ///       CLI use (CLI at priority 0 cannot pre-empt telemetry task at priority 1 while
    ///       the lock is held). Write gaps during enumeration are expected and harmless.
    void listLogs();
    void dumpLog(int index);
    void deleteLog(int index);

    /// @brief Erase the entire LittleFS filesystem. Stops logging first. Use with care.
    void reset() override
    {
        // Stop any active logging session before formatting to prevent the telemetry
        // task from writing to an invalidated file handle after the format completes.
        if (!stopLogging())
        {
            LOG_ERR("Flash reset aborted: active log could not be closed safely");
            return;
        }

        if (!_fileMutex) return;

        // Use portMAX_DELAY so the format only begins after any in-progress write
        // completes. The format itself takes up to several seconds; the telemetry
        // task will gracefully time out on _fileMutex during this period and skip
        // write cycles (acceptable \u2014 the FS is being erased anyway).
        SemaphoreLock lock(_fileMutex, portMAX_DELAY);
        if (!lock.acquired()) return;
        if (!LittleFS.format()) { LOG_ERR("LittleFS format failed"); }
    }

private:
    static void telemetryTask(void* pvParameters);

    // Helpers — caller must hold _fileMutex before calling either function.
    /// @note Caller must hold _fileMutex. Traverses LittleFS root once; returns
    ///       (max_index + 1), or 0 when the filesystem contains no log files.
    int  findNextFlightLogIndex();
    static void formatCSVHeader(char* buf, size_t bufSize);
    static size_t formatCSVRow(char* buf, size_t bufSize, unsigned long ts, const TelemetryData& d);

    float             _intervalMs;
    TaskHandle_t      _taskHandle = nullptr; ///< Handle for telemetryTask; stored to allow clean teardown
    SemaphoreHandle_t _dataMutex;       ///< Protects _pendingData (fast operations)
    SemaphoreHandle_t _fileMutex;       ///< Protects file I/O (slow operations)
    TelemetryData     _pendingData{};   ///< Updated by publish()
    TelemetryData     _writeBuffer{};   ///< Used by task for writing
    unsigned long     _lastFlushMs;
    File              _logFile;
    std::atomic<bool> _isLogging;
    char              _currentFilename[32]{}; ///< Current log filename, e.g. "/log_000.csv"
};

#endif //ARDUFLITE_FLASH_TELEMETRY_H
