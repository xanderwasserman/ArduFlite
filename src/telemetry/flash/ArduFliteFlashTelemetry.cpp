// src/telemetry/flash/ArduFliteFlashTelemetry.cpp
//
// ArduFlite - Advanced Flight Controller Framework
// Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
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
  : _intervalMs(1000.0f / constrain(frequencyHz, 0.1f, 200.0f)),
    _dataMutex(nullptr),
    _fileMutex(nullptr),
    _lastFlushMs(0),
    _isLogging(false)
{
}

ArduFliteFlashTelemetry::~ArduFliteFlashTelemetry()
{
    // Delete the background task first so it cannot access _logFile or the mutexes
    // after they are destroyed below. On single-core ESP32-C3 vTaskDelete() removes
    // the task from the scheduler immediately — it will not run again, and since only
    // one task runs at a time it cannot be mid-write here.
    //
    // SINGLE-CORE ASSUMPTION (load-bearing): on a dual-core port (ESP32-S3/classic),
    // vTaskDelete() of a task pinned to the OTHER core is not synchronous, so the
    // telemetry task could still be inside its _fileMutex write scope when we close
    // _logFile below — a use-after-free. Revisit this teardown before any dual-core
    // port (e.g. join via a "task exited" flag the task sets just before returning).
    // Note also that vTaskDelete() does not unwind the deleted task's C++ stack, so a
    // SemaphoreLock it held is never given back — harmless here only because we delete
    // the mutexes next.
    if (_taskHandle)
    {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }

    if (_fileMutex)
    {
        // The telemetryTask was deleted above, so _fileMutex is uncontested.
        // Access _isLogging and _logFile directly without acquiring a lock.
        if (_isLogging)
        {
            _logFile.flush();
            _logFile.close();
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
    // Idempotency guard — a second begin() call would leak the existing mutexes
    // and spawn a second task that races on the same _pendingData / _logFile.
    if (_dataMutex)
    {
        LOG_WARN("FlashTelemetry::begin() called more than once — ignoring");
        return;
    }

    if (!LittleFS.begin())
    {
        LOG_ERR("LittleFS mount failed; formatting...");
        if (!LittleFS.format())
        {
            LOG_ERR("LittleFS format failed — cannot mount");
            return;
        }
        if (!LittleFS.begin())
        {
            LOG_ERR("LittleFS mount failed after format!");
            return;
        }
    }

    _dataMutex = xSemaphoreCreateMutex();
    _fileMutex = xSemaphoreCreateMutex();
    if (!_dataMutex || !_fileMutex)
    {
        LOG_ERR("Failed to create flash telemetry mutexes");
        // Roll back any mutexes that were created so the object stays in a
        // fully uninitialised state. Without this, a second begin() call would
        // see _dataMutex != nullptr, hit the idempotency guard, and silently
        // return — permanently blocking recovery without an error message.
        if (_dataMutex) { vSemaphoreDelete(_dataMutex); _dataMutex = nullptr; }
        if (_fileMutex) { vSemaphoreDelete(_fileMutex); _fileMutex = nullptr; }
        return;
    }

    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    LOG_INF("LittleFS TotalBytes: %u", (unsigned)total);
    LOG_INF("LittleFS UsedBytes:  %u\n\n", (unsigned)used);

    // Start background task — store handle so the destructor can stop it cleanly.
    if (xTaskCreate(
        telemetryTask,
        "FlashTelTask",
        8192,
        this,
        1,
        &_taskHandle
    ) != pdPASS)
    {
        LOG_ERR("FlashTelemetry: failed to create telemetry task");
        vSemaphoreDelete(_dataMutex); _dataMutex = nullptr;
        vSemaphoreDelete(_fileMutex); _fileMutex = nullptr;
    }
}

void ArduFliteFlashTelemetry::publish(const TelemetryData& telemData)
{
    if (!_dataMutex) return;

    SemaphoreLock lock(_dataMutex);
    if (!lock.acquired()) return;
    _pendingData = telemData;
}

int ArduFliteFlashTelemetry::findNextFlightLogIndex()
{
    // Policy: indices grow monotonically as (max_index + 1) until log_999.csv exists;
    // only then do we reuse the lowest free gap. Deleting a mid-range log does NOT
    // reclaim its index until the space is full, which keeps log ordering intuitive.
    // -1 sentinel → returns 0 (first log index) when the directory is empty.
    static constexpr int MAX_LOG_INDEX = 999;

    // First pass: track only the highest index in use (no large stack buffer).
    int maxIdx = -1;
    {
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while (file)
        {
            // Avoid String heap allocation — use LittleFS filename pointer directly.
            const char* p = file.name();
            if (*p == '/') ++p;
            int idx;
            if (sscanf(p, "log_%03d.csv", &idx) == 1 && idx >= 0 && idx <= MAX_LOG_INDEX)
            {
                maxIdx = max(maxIdx, idx);
            }
            file = root.openNextFile();
        }
        root.close();
    }

    // Common case: room remains above the highest index — done, no occupancy scan.
    if (maxIdx < MAX_LOG_INDEX)
    {
        return maxIdx + 1;
    }

    // Rare case: log_999.csv exists. Scan once more, this time recording occupancy,
    // to reuse the lowest free index. The 1 KB array lives only on this path so the
    // common case never pays its stack cost.
    bool used[MAX_LOG_INDEX + 1] = {};
    {
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while (file)
        {
            const char* p = file.name();
            if (*p == '/') ++p;
            int idx;
            if (sscanf(p, "log_%03d.csv", &idx) == 1 && idx >= 0 && idx <= MAX_LOG_INDEX)
            {
                used[idx] = true;
            }
            file = root.openNextFile();
        }
        root.close();
    }

    for (int idx = 0; idx <= MAX_LOG_INDEX; ++idx)
    {
        if (!used[idx]) return idx;
    }

    LOG_ERR("findNextFlightLogIndex: log index space exhausted; delete old logs first");
    return -1;
}

bool ArduFliteFlashTelemetry::startLogging()
{
    using namespace FlashTelemetryConfig;
    if (!_fileMutex) return false;

    // Bounded timeout — portMAX_DELAY would block a CLI task (priority 0) indefinitely
    // if the telemetry task (priority 1) holds _fileMutex. LittleFS writes complete
    // in well under 500 ms under normal conditions.
    constexpr TickType_t LOCK_TIMEOUT_MS = 500;
    SemaphoreLock lock(_fileMutex, LOCK_TIMEOUT_MS);
    if (!lock.acquired())
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

    // Auto-purge oldest log(s) if flash storage is critically low.
    // Both find and delete run under _fileMutex to prevent concurrent directory
    // traversal from listLogs() or dumpLog() from racing with the purge loop.
    // At 10 Hz each log row is ~175 bytes; 300 KB provides ~3 minutes of headroom.
    constexpr size_t MIN_FREE_BYTES     = 300UL * 1024UL;
    constexpr int    MAX_LOG_INDICES    = 50;   ///< Stack cap on tracked log entries
    constexpr int    MAX_PURGE_ATTEMPTS = 20;   ///< Safety cap against filesystem errors

    // Safe subtraction: guard against corrupt filesystem where usedBytes > totalBytes.
    size_t freeBytes = (LittleFS.usedBytes() <= LittleFS.totalBytes())
                       ? (LittleFS.totalBytes() - LittleFS.usedBytes()) : 0;

    if (freeBytes < MIN_FREE_BYTES)
    {
        // Single directory scan to collect all log indices — avoids calling
        // findOldestFlightLogIndex() (a full O(N) scan) on every purge iteration,
        // which would make the total purge O(N²). Instead, sort once and iterate.
        int logIndices[MAX_LOG_INDICES];
        int logCount = 0;
        {
            File root = LittleFS.open("/");
            File f    = root.openNextFile();
            while (f && logCount < MAX_LOG_INDICES)
            {
                const char* p = f.name();
                if (*p == '/') ++p;
                int idx;
                if (sscanf(p, "log_%03d.csv", &idx) == 1)
                {
                    logIndices[logCount++] = idx;
                }
                f = root.openNextFile();
            }
            if (logCount >= MAX_LOG_INDICES && f)
            {
                // More log files exist beyond the scan cap — the oldest ones
                // outside this range will not be considered for purge this cycle.
                LOG_WARN("startLogging: >%d logs on filesystem — scan capped; some old logs may not be auto-purged", MAX_LOG_INDICES);
            }
            root.close();
        }

        // Insertion sort ascending (oldest = smallest index first).
        // logCount ≤ MAX_LOG_INDICES, so O(N²) is negligible here.
        for (int i = 1; i < logCount; ++i)
        {
            int key = logIndices[i], j = i - 1;
            while (j >= 0 && logIndices[j] > key) { logIndices[j + 1] = logIndices[j]; --j; }
            logIndices[j + 1] = key;
        }

        for (int pi = 0; pi < logCount && freeBytes < MIN_FREE_BYTES; ++pi)
        {
            if (pi >= MAX_PURGE_ATTEMPTS)
            {
                LOG_ERR("Auto-purge aborted after %d attempts — filesystem may be corrupt.", MAX_PURGE_ATTEMPTS);
                return false;
            }
            char purgeFn[32];
            snprintf(purgeFn, sizeof(purgeFn), "/log_%03d.csv", logIndices[pi]);
            if (!LittleFS.remove(purgeFn))
            {
                LOG_ERR("Auto-purge failed to delete %s — aborting.", purgeFn);
                return false;
            }
            freeBytes = (LittleFS.usedBytes() <= LittleFS.totalBytes())
                        ? (LittleFS.totalBytes() - LittleFS.usedBytes()) : 0;
            LOG_WARN("Auto-purged %s — %.1f KB free", purgeFn, freeBytes / 1024.0f);
        }

        if (freeBytes < MIN_FREE_BYTES)
        {
            LOG_ERR("Flash full and no purgeable logs — cannot start logging.");
            return false;
        }
    }

    int idx = findNextFlightLogIndex();
    if (idx < 0)
    {
        return false;
    }
    snprintf(_currentFilename, sizeof(_currentFilename), "/log_%03d.csv", idx);

    _logFile = LittleFS.open(_currentFilename, FILE_WRITE);
    if (_logFile)
    {
        char header[MAX_ROW_BUFFER];
        formatCSVHeader(header, sizeof(header));
        _logFile.write((const uint8_t*)header, strlen(header));
        _logFile.flush();
        _lastFlushMs = millis();
        _isLogging = true;
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
    constexpr TickType_t LOCK_TIMEOUT_MS = 500;
    SemaphoreLock lock(_fileMutex, LOCK_TIMEOUT_MS);
    if (!lock.acquired())
    {
        LOG_ERR("stopLogging: could not acquire _fileMutex within 500 ms — log may not be flushed");
        return false;
    }

    if (_isLogging)
    {
        _logFile.flush();
        _logFile.close();
        _isLogging = false;
        LOG_INF("Logging stopped: %s", _currentFilename);
    }

    return true;
}

void ArduFliteFlashTelemetry::listLogs()
{
    if (!_fileMutex) return;

    if (_isLogging)
    {
        LOG_WARN("listLogs() called while logging is active — write gaps will occur during enumeration");
    }

    // Diagnostic commands use a generous bounded timeout — these run from CLI
    // (priority 0); the telemetry task (priority 1) only holds _fileMutex briefly.
    constexpr TickType_t DIAG_LOCK_TIMEOUT_MS = 2000;
    SemaphoreLock lock(_fileMutex, DIAG_LOCK_TIMEOUT_MS);
    if (!lock.acquired())
    {
        LOG_ERR("listLogs: could not acquire _fileMutex within 2000 ms");
        return;
    }

    LOG("Available logs:");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file)
    {
        // Avoid String heap allocation — match log_NNN.csv via sscanf.
        const char* name = file.name();
        const char* p    = (*name == '/') ? name + 1 : name;
        int idx;
        if (sscanf(p, "log_%03d.csv", &idx) == 1)
            LOG("  log_%03d.csv", idx);
        file = root.openNextFile();
    }
    root.close();
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
    File f;
    {
        constexpr TickType_t DIAG_LOCK_TIMEOUT_MS = 2000;
        SemaphoreLock lock(_fileMutex, DIAG_LOCK_TIMEOUT_MS);
        if (!lock.acquired())
        {
            LOG_ERR("dumpLog: could not acquire _fileMutex within 2000 ms");
            return;
        }
        f = LittleFS.open(fn, FILE_READ);
    }  // _fileMutex released here — Serial I/O proceeds without holding the lock

    if (!f)
    {
        LOG_ERR("Failed to open %s", fn);
        return;
    }

    // BEGIN marker + blank line
    LOG_N("\n--- BEGIN %s ---\n\n", fn);

    // Read and print line by line using a fixed stack buffer to avoid
    // repeated heap alloc/free from Arduino String on every row.
    char lineBuf[FlashTelemetryConfig::MAX_ROW_BUFFER];
    while (f.available())
    {
        int n = f.readBytesUntil('\n', lineBuf, sizeof(lineBuf) - 1);
        lineBuf[n] = '\0';
        LOG("%s", lineBuf);
    }

    // END marker
    LOG_N("\n--- END %s ---\n", fn);
    f.close();
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

    constexpr TickType_t DIAG_LOCK_TIMEOUT_MS = 2000;
    SemaphoreLock lock(_fileMutex, DIAG_LOCK_TIMEOUT_MS);
    if (!lock.acquired())
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

        // 2) Format CSV row outside any mutex.
        // If snprintf truncates (len >= bufSize), the row is missing its trailing '\n',
        // which would concatenate subsequent rows onto the same CSV line and corrupt
        // the log. Skip the row and emit an error rather than writing a partial record.
        size_t len = formatCSVRow(rowBuf, sizeof(rowBuf), ts, self->_writeBuffer);
        const bool rowTruncated = (len >= sizeof(rowBuf));
        if (rowTruncated)
        {
            LOG_ERR("FlashTel: CSV row truncated (%u bytes) — row skipped to preserve log integrity",
                    (unsigned)len);
        }

        // 3) SLOW: Write to flash under file mutex (doesn't block publish)
        if (!rowTruncated && self->_fileMutex)
        {
            // Intentional short timeout: if startLogging/reset/dumpLog holds _fileMutex,
            // silently drop this write cycle rather than blocking the task.
            SemaphoreLock lock(self->_fileMutex, MUTEX_TIMEOUT_MS);
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

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void ArduFliteFlashTelemetry::formatCSVHeader(char* buf, size_t bufSize)
{
    using namespace FlashTelemetryConfig;
    // 33 columns are logged (1 timestamp + 27 sensor/control floats + 5 ints).
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
        "imu_snapshot_retries,imu_snapshot_max_retries,imu_snapshot_retry_limit_hits\n";
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
    // Logs the same 33 columns defined in formatCSVHeader(). Fields intentionally
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
        "%lu,%lu,%lu\n",
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
        (unsigned long)d.imu_snapshot_retry_limit_hits
    );
    return (snprintfRet < 0) ? bufSize : (size_t)snprintfRet;
}
