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

#include "src/telemetry/PeriodicTelemetryBackend.h"
#include "src/utils/Logging.h"

#include <atomic>
#include <chrono>
#include <mutex>

class ArduFliteFlashTelemetry final : public PeriodicTelemetryBackend
{
public:
    explicit ArduFliteFlashTelemetry(float frequencyHz = 10.0f);
    ~ArduFliteFlashTelemetry() override;

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

    /// @brief Erase all stored logs. Stops logging first. Use with care.
    void reset() override
    {
        // Stop any active logging session before formatting to prevent the telemetry
        // task from writing to an invalidated file handle after the format completes.
        if (!stopLogging())
        {
            LOG_ERR("Flash reset aborted: active log could not be closed safely");
            return;
        }

        if (_fileMutex == nullptr) { return; }

        // An UNBOUNDED wait, so the format only begins after any in-progress
        // write completes. The format itself takes several seconds; the
        // telemetry task times out on _fileMutex meanwhile and skips write
        // cycles, which is acceptable because the filesystem is being erased.
        std::unique_lock lock(*_fileMutex);
        if (!_store || _store->formatAll() != arduflite::Status::Ok)
        {
            LOG_ERR("Log store format failed");
        }
    }

private:
    /// Mounts the log store and takes the file mutex, before the task starts.
    bool onBegin() override;
    void runLoop() override;

    // Helpers — caller must hold _fileMutex before calling either function.
    /// @note Caller must hold _fileMutex. Traverses LittleFS root once; returns
    ///       (max_index + 1), or 0 when the filesystem contains no log files.
    int  findNextFlightLogIndex();

    /// Cap on how many log indices one directory scan will track.
    static constexpr size_t kMaxTrackedLogs = 50;

    /// Reads the indices present on the filesystem. The RULE for choosing and
    /// purging them lives in drivers::LogRotationPolicy, where it is testable
    /// without a filesystem.
    size_t collectLogIndices(int* out, size_t maxEntries);

    /// Storage, owned by Board. All file work goes through it.
    arduflite::device::LogStore* _store = nullptr;

    /// Latches on the first failed append so a full medium reports once,
    /// not fifty times a second. Cleared by startLogging().
    bool _writeFailed = false;
    static void formatCSVHeader(char* buf, size_t bufSize);
    static size_t formatCSVRow(char* buf, size_t bufSize, unsigned long ts, const TelemetryData& d);

    /// How long a diagnostic command will wait for the file lock. Generous:
    /// these run from the CLI on the ground, where waiting beats failing.
    static constexpr std::chrono::milliseconds kDiagLockTimeout{ 2000 };
    /// How long start/stop will wait. Shorter: these can be hit in flight.
    static constexpr std::chrono::milliseconds kControlLockTimeout{ 500 };

    /// Protects file I/O, which is slow. The base's mutex guards the published
    /// sample and is taken only briefly, so the two are kept separate: a CLI
    /// command holding this one for seconds must not block publish().
    arduflite::hal::Mutex* _fileMutex = nullptr;
    TelemetryData     _writeBuffer{};   ///< The task's working copy
    /// Monotonic microseconds from hal::Clock, not Arduino millis(). The clock
    /// is 64-bit, so unlike millis() it does not wrap after 49 days — which is
    /// longer than any flight but not longer than a bench session left running.
    std::uint64_t     _lastFlushUs = 0;
    std::atomic<bool> _isLogging;
    char              _currentFilename[32]{}; ///< Current log filename, e.g. "/log_000.csv"
};

#endif //ARDUFLITE_FLASH_TELEMETRY_H
