// src/telemetry/flash/ArduFliteFlashTelemetry.cpp
//
// ArduFlite - Advanced Flight Controller Framework
// Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
//
// Licensed under the MIT License. See LICENSE file for details.
//

#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "src/utils/Logging.h"

#include <FS.h>

#include <algorithm>
#include <mutex>

#include "src/core/LogRotationPolicy.h"
#include "src/hal/board/Board.h"

namespace FlashTelemetryConfig {
    constexpr std::uint64_t FLUSH_INTERVAL_US = 500'000;   ///< Flush to flash every 500 ms
    constexpr size_t MAX_ROW_BUFFER = 600;             ///< Max CSV row size in bytes
}

ArduFliteFlashTelemetry::ArduFliteFlashTelemetry(float frequencyHz)
  : PeriodicTelemetryBackend("FlashTelTask", frequencyHz, 8192),
    _isLogging(false)
{
}

ArduFliteFlashTelemetry::~ArduFliteFlashTelemetry()
{
    // Ask the loop to leave BEFORE closing anything. Taking _fileMutex below is
    // what orders this against a write already in progress.
    requestTaskStop();

    if (_fileMutex != nullptr)
    {
        // Unbounded: a flight log worth closing is worth waiting for.
        std::unique_lock lock(*_fileMutex);
        if (_isLogging && _store != nullptr)
        {
            (void)_store->flush();
            (void)_store->closeSession();
            _isLogging = false;
        }
    }

    // Pool-owned; nothing reclaims entries (ADR-011, no heap after boot).
    _fileMutex = nullptr;
}

bool ArduFliteFlashTelemetry::onBegin()
{
    _store = &arduflite::board::Board::instance().logs();
    if (_store->begin() != arduflite::Status::Ok)
    {
        LOG_ERR("Log store failed to mount - flash logging disabled");
        _store = nullptr;
        return false;
    }

    auto fileMutex = arduflite::board::Board::instance().allocMutex();
    if (!fileMutex)
    {
        LOG_ERR("FlashTelemetry: no file mutex available - flash logging disabled");
        _store = nullptr;
        return false;
    }
    _fileMutex = fileMutex.value();

    uint32_t used = 0, total = 0;
    (void)_store->usage(used, total);
    LOG_INF("Log store: %u bytes used of %u", (unsigned)used, (unsigned)total);
    return true;
}


/**
 * @brief Collect the log indices present on the filesystem.
 *
 * Split from the allocation rule so the rule itself — monotonic growth, then
 * lowest-gap reuse — is testable without a filesystem. See
 * drivers::LogRotationPolicy and test_log_rotation.cpp.
 *
 * @return how many were written to `out`, capped at maxEntries.
 */
size_t ArduFliteFlashTelemetry::collectLogIndices(int* out, size_t maxEntries)
{
    if (!_store) { return 0; }

    uint16_t indices[kMaxTrackedLogs];
    const size_t capped = (maxEntries < kMaxTrackedLogs) ? maxEntries : kMaxTrackedLogs;
    const size_t count  = _store->listSessions(indices, capped);

    for (size_t i = 0; i < count; ++i) { out[i] = indices[i]; }

    if (count >= capped)
    {
        LOG_WARN("collectLogIndices: at least %u logs present - scan capped; "
                 "any older ones are not considered for purge", (unsigned)capped);
    }
    return count;
}

/// Free space. The store already guards used > total.
static uint32_t freeBytesOf(arduflite::device::LogStore* store)
{
    if (!store) { return 0; }
    uint32_t used = 0, total = 0;
    if (store->usage(used, total) != arduflite::Status::Ok) { return 0; }
    return (used <= total) ? (total - used) : 0u;
}

int ArduFliteFlashTelemetry::findNextFlightLogIndex()
{
    int indices[kMaxTrackedLogs];
    const size_t count = collectLogIndices(indices, kMaxTrackedLogs);

    const int idx = arduflite::drivers::LogRotationPolicy::nextIndex(indices, count);
    if (idx < 0)
    {
        LOG_ERR("findNextFlightLogIndex: log index space exhausted; delete old logs first");
    }
    return idx;
}

bool ArduFliteFlashTelemetry::startLogging()
{
    using namespace FlashTelemetryConfig;
    if (!_fileMutex) return false;

    // Bounded timeout — portMAX_DELAY would block a CLI task (priority 0) indefinitely
    // if the telemetry task (priority 1) holds _fileMutex. LittleFS writes complete
    // in well under 500 ms under normal conditions.
    std::unique_lock lock(*_fileMutex, kControlLockTimeout);
    if (!lock.owns_lock())
    {
        LOG_ERR("startLogging: could not acquire _fileMutex within 500 ms — log not started");
        return false;
    }

    if (_isLogging)
    {
        // Already recording — return true: a log IS active for this flight, which is
        // what callers care about. The warning flags the redundant call, not a failure.
        LOG_WARN("startLogging: already logging to %s — call stopLogging() first", _currentFilename);
        return true;
    }

    // Auto-purge the oldest logs if flash is critically low. Both the scan and
    // the deletes run under _fileMutex so a concurrent listLogs() or dumpLog()
    // cannot traverse the directory while it is changing underneath them.
    //
    // The RULE lives in drivers::LogRotationPolicy and is tested on the host;
    // what remains here is the filesystem work it decides on.
    {
        arduflite::drivers::LogRotationPolicy policy;
        uint32_t freeBytes = freeBytesOf(_store);

        if (freeBytes < policy.minFreeBytes)
        {
            int indices[kMaxTrackedLogs];
            const size_t count = collectLogIndices(indices, kMaxTrackedLogs);
            arduflite::drivers::LogRotationPolicy::sortAscending(indices, count);

            // Re-measure after every delete rather than trusting an estimate:
            // the policy decides HOW MANY based on an average, but the actual
            // reclaim varies with log length, and stopping as soon as the real
            // figure clears the threshold deletes the fewest flights.
            for (size_t i = 0; i < count && freeBytes < policy.minFreeBytes; ++i)
            {
                if (i >= (size_t)policy.maxPurgeAttempts)
                {
                    LOG_ERR("Auto-purge aborted after %d attempts - filesystem may be corrupt.",
                            policy.maxPurgeAttempts);
                    return false;
                }

                if (_store->removeSession((uint16_t)indices[i]) != arduflite::Status::Ok)
                {
                    LOG_ERR("Auto-purge failed to delete log_%03d.csv - aborting.", indices[i]);
                    return false;
                }
                char purgeFn[32];
                snprintf(purgeFn, sizeof(purgeFn), "/log_%03d.csv", indices[i]);

                freeBytes = freeBytesOf(_store);
                LOG_WARN("Auto-purged %s - %.1f KB free", purgeFn, freeBytes / 1024.0f);
            }

            if (freeBytes < policy.minFreeBytes)
            {
                LOG_ERR("Flash full and no purgeable logs - cannot start logging.");
                return false;
            }
        }
    }

    int idx = findNextFlightLogIndex();
    if (idx < 0)
    {
        return false;
    }
    snprintf(_currentFilename, sizeof(_currentFilename), "/log_%03d.csv", idx);

    if (_store->openSession((uint16_t)idx) == arduflite::Status::Ok)
    {
        char header[MAX_ROW_BUFFER];
        formatCSVHeader(header, sizeof(header));
        (void)_store->append(header, strlen(header));
        (void)_store->flush();
        // Reset the flush deadline so a session that starts just after a flush
        // does not immediately flush again.
        _lastFlushUs = static_cast<std::uint64_t>(
            arduflite::board::Board::instance().clock().now().time_since_epoch().count());
        _isLogging   = true;

        // Clear the latch: a medium that was full when the LAST log ran may
        // have been purged since, and leaving it set would silence the failure
        // report for every remaining flight of this boot.
        _writeFailed = false;

        LOG_INF("Logging started: %s", _currentFilename);
        return true;
    }

    LOG_ERR("Failed to open %s", _currentFilename);
    return false;
}

bool ArduFliteFlashTelemetry::stopLogging()
{
    if (!_fileMutex) return false;

    // Bounded timeout — stopLogging() is called from CLI (priority 0) or landing
    // callbacks. The telemetry task (priority 1) only holds _fileMutex briefly
    // during LittleFS writes; 500 ms provides ample margin.
    std::unique_lock lock(*_fileMutex, kControlLockTimeout);
    if (!lock.owns_lock())
    {
        LOG_ERR("stopLogging: could not acquire _fileMutex within 500 ms — log may not be flushed");
        return false;
    }

    if (_isLogging)
    {
        (void)_store->flush();
        (void)_store->closeSession();
        _isLogging = false;
        LOG_INF("Logging stopped: %s", _currentFilename);
    }

    return true;
}

void ArduFliteFlashTelemetry::listLogs()
{
    if (!_fileMutex) return;
    if (!_store) { LOG_ERR("listLogs: log store unavailable"); return; }

    if (_isLogging)
    {
        LOG_WARN("listLogs() called while logging is active - write gaps will occur during enumeration");
    }

    // Diagnostic commands use a generous bounded timeout - these run from the
    // CLI (priority 1); the telemetry task only holds _fileMutex briefly.
    std::unique_lock lock(*_fileMutex, kDiagLockTimeout);
    if (!lock.owns_lock())
    {
        LOG_ERR("listLogs: could not acquire _fileMutex within 2000 ms");
        return;
    }

    uint16_t indices[kMaxTrackedLogs];
    const size_t count = _store->listSessions(indices, kMaxTrackedLogs);

    // Oldest first, matching how the purge walks them and how anyone reading a
    // directory listing expects log numbers to run.
    int ordered[kMaxTrackedLogs];
    for (size_t i = 0; i < count; ++i) { ordered[i] = indices[i]; }
    arduflite::drivers::LogRotationPolicy::sortAscending(ordered, count);

    LOG_N("Flight logs (%u):\n", (unsigned)count);
    for (size_t i = 0; i < count; ++i)
    {
        LOG_N("  log_%03d.csv\n", ordered[i]);
    }

    uint32_t used = 0, total = 0;
    (void)_store->usage(used, total);
    LOG_N("Storage: %u / %u bytes used\n", (unsigned)used, (unsigned)total);
}

void ArduFliteFlashTelemetry::dumpLog(int index)
{
    if (!_fileMutex) return;

    // Validate index before constructing the path — out-of-range indices produce
    // a malformed filename that LittleFS will silently refuse to open.
    if (index < 0 || index > 999)
    {
        LOG_ERR("dumpLog: invalid index %d (must be 0\u2013999)", index);
        return;
    }

    if (_isLogging)
    {
        LOG_WARN("dumpLog() called while logging is active — write gaps will occur during the dump");
    }

    char fn[32];
    snprintf(fn, sizeof(fn), "/log_%03d.csv", index);

    // Open the file under the mutex, then release it before the Serial dump.
    // Holding _fileMutex across blocking Serial I/O would starve the telemetry
    // task's 5 ms write window for the entire dump duration.
    {
        std::unique_lock lock(*_fileMutex, kDiagLockTimeout);
        if (!lock.owns_lock())
        {
            LOG_ERR("dumpLog: could not acquire _fileMutex within 2000 ms");
            return;
        }
        if (!_store)
        {
            LOG_ERR("dumpLog: log store unavailable");
            return;
        }
    }  // _fileMutex released here — console I/O proceeds without holding the lock

    LOG_N("\n--- BEGIN %s ---\n\n", fn);

    // Streamed in chunks, not read whole: a flight log runs to hundreds of
    // kilobytes and there is no buffer that size. A fixed stack buffer also
    // avoids the per-row heap churn Arduino String would cause.
    // Chunks are written to the console raw: they split rows at arbitrary
    // byte offsets, so a logger call — which appends a newline and caps the
    // line length — would corrupt the CSV.
    auto&    console = arduflite::board::Board::instance().console();
    char     chunk[FlashTelemetryConfig::MAX_ROW_BUFFER];
    size_t   offset = 0;
    for (;;)
    {
        size_t length = 0;
        if (_store->readSession((uint16_t)index, chunk, sizeof(chunk), offset, length)
                != arduflite::Status::Ok)
        {
            LOG_ERR("Failed to read %s", fn);
            break;
        }
        if (length == 0) { break; }   // end of data

        console.write(chunk, length);
        offset += length;
    }

    LOG_N("\n--- END %s ---\n", fn);
}

void ArduFliteFlashTelemetry::deleteLog(int index)
{
    if (!_fileMutex) return;

    // Validate index before constructing the path — out-of-range indices produce
    // a malformed filename that LittleFS silently refuses, masking the caller error.
    if (index < 0 || index > 999)
    {
        LOG_ERR("deleteLog: invalid index %d (must be 0\u2013999)", index);
        return;
    }

    std::unique_lock lock(*_fileMutex, kDiagLockTimeout);
    if (!lock.owns_lock())
    {
        LOG_ERR("deleteLog: could not acquire _fileMutex within 2000 ms");
        return;
    }

    char fn[32];
    snprintf(fn, sizeof(fn), "/log_%03d.csv", index);

    // Refuse to delete the file whose handle is currently held open by the telemetry
    // task — LittleFS behaviour on deleting an open file is implementation-defined
    // and can corrupt flash or silently break subsequent writes.
    if (_isLogging && strcmp(fn, _currentFilename) == 0)
    {
        LOG_ERR("deleteLog: cannot delete active log %s — call stopLogging() first", fn);
        return;
    }

    if (_store && _store->removeSession((uint16_t)index) == arduflite::Status::Ok)
    {
        LOG_INF("Deleted %s", fn);
    }
    else
    {
        LOG_ERR("Failed to delete %s", fn);
    }
}

void ArduFliteFlashTelemetry::runLoop()
{
    using namespace FlashTelemetryConfig;
    char rowBuf[MAX_ROW_BUFFER];

    auto& board     = arduflite::board::Board::instance();
    auto& scheduler = board.scheduler();
    // Named `boardClock`, not `clock`: <ctime> declares a ::clock() that an
    // unqualified `clock` binds to happily, and the error it eventually gives
    // points at the member access rather than the name.
    const auto& boardClock = board.clock();

    // sleepUntil, not sleepFor: fixed cadence measured from the start of each
    // iteration, so flash-write time does not accumulate into drift.
    std::uint64_t lastWake = 0;
    const auto period =
        std::chrono::milliseconds{ static_cast<std::int64_t>(intervalMs()) };

    while (shouldRun())
    {
        const std::uint64_t nowUs = static_cast<std::uint64_t>(
            boardClock.now().time_since_epoch().count());
        const unsigned long ts = static_cast<unsigned long>(nowUs / 1000);

        // 1) FAST: take a copy of the published sample. On a timeout the
        //    previous _writeBuffer is reused — the log wants an unbroken row
        //    cadence, and the timestamp makes a repeat visible.
        (void)snapshot(_writeBuffer);

        // 2) Format CSV row outside any mutex.
        // If snprintf truncates (len >= bufSize), the row is missing its trailing '\n',
        // which would concatenate subsequent rows onto the same CSV line and corrupt
        // the log. Skip the row and emit an error rather than writing a partial record.
        size_t len = formatCSVRow(rowBuf, sizeof(rowBuf), ts, _writeBuffer);
        const bool rowTruncated = (len >= sizeof(rowBuf));
        if (rowTruncated)
        {
            LOG_ERR("FlashTel: CSV row truncated (%u bytes) — row skipped to preserve log integrity",
                    (unsigned)len);
        }

        // 3) SLOW: Write to flash under file mutex (doesn't block publish)
        if (!rowTruncated && _fileMutex)
        {
            // Intentional short timeout: if startLogging/reset/dumpLog holds _fileMutex,
            // silently drop this write cycle rather than blocking the task.
            std::unique_lock lock(*_fileMutex, kTelemetryLockTimeout);
            if (lock.owns_lock() && _isLogging && _store && _store->isOpen())
            {
                const arduflite::Status status = _store->append(rowBuf, len);
                if (status != arduflite::Status::Ok)
                {
                    // NoSpace means the medium filled mid-flight. Every row from
                    // here is lost, so say it once rather than per row at 50 Hz.
                    if (!_writeFailed)
                    {
                        LOG_ERR("Flash write failed (%s) - log is now truncated",
                                arduflite::toString(status));
                        _writeFailed = true;
                    }
                }
                else if (nowUs - _lastFlushUs >= FLUSH_INTERVAL_US)
                {
                    (void)_store->flush();
                    _lastFlushUs = nowUs;
                }
            }
        }

        scheduler.sleepUntil(lastWake, period);
    }
}

void ArduFliteFlashTelemetry::formatCSVHeader(char* buf, size_t bufSize)
{
    using namespace FlashTelemetryConfig;
    // 38 columns are logged (1 timestamp + 31 sensor/control floats + 6 ints).
    // New columns are APPENDED, never inserted: the log is CSV with a named
    // header, so appending is backward compatible for anything that reads by
    // column name (tools/data_analysis reads with pandas; tools/csv_viewer
    // reads the header). Inserting would silently shift every older parser.
    // The following TelemetryData fields are intentionally deferred and NOT included:
    //   battery_voltage, battery_current, battery_percent  — hardware not yet wired
    //   gps_lat, gps_lon, gps_alt, gps_speed, gps_hdop     — GPS module optional
    //   armed, in_failsafe, link_quality, link_rssi         — populated elsewhere
    // Add those columns here AND in formatCSVRow() once live data is available,
    // and update the CSV parsing in tools/data_analysis/ accordingly.
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
        "altitude,climb_rate,flight_state,flight_mode,"
        "imu_snapshot_retries,imu_snapshot_max_retries,imu_snapshot_retry_limit_hits,"
        "mag_x,mag_y,mag_z,mag_heading,mag_field,mag_valid\n";
    // bufSize should be at least MAX_ROW_BUFFER (600)
    int hdrRet = snprintf(buf, bufSize, "%s", hdr);
    if (hdrRet < 0 || (size_t)hdrRet >= bufSize)
    {
        LOG_ERR("formatCSVHeader: header truncated (%d bytes needed, %u available) — CSV column names will be incomplete",
                hdrRet, (unsigned)bufSize);
    }
}

size_t ArduFliteFlashTelemetry::formatCSVRow(
    char* buf, size_t bufSize,
    unsigned long ts,
    const TelemetryData& d
)
{
    // Logs the same 38 columns defined in formatCSVHeader(). Fields intentionally
    // omitted (battery_*, gps_*, armed, in_failsafe, link_*) are documented there.
    // snprintf returns -1 on encoding error; casting negative int to size_t yields
    // SIZE_MAX which the caller treats as truncation (correct skip), but is
    // technically implementation-defined. Capture and check explicitly.
    int snprintfRet = snprintf(buf, bufSize,
        "%lu,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.2f,"
        "%.2f,"
        "%d,"
        "%d,"
        "%lu,%lu,%lu,"
        "%.2f,%.2f,%.2f,%.1f,%.2f,%d\n",
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
        d.flight_mode,
        (unsigned long)d.imu_snapshot_retries,
        (unsigned long)d.imu_snapshot_max_retries,
        (unsigned long)d.imu_snapshot_retry_limit_hits,
        // magnetometer — instrumentation, see TelemetryData
        d.mag.x, d.mag.y, d.mag.z,
        d.mag_heading,
        d.mag_field,
        d.mag_valid ? 1 : 0
    );
    return (snprintfRet < 0) ? bufSize : (size_t)snprintfRet;
}
