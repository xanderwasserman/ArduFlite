/**
 * PeriodicTelemetryBackend.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * The lifecycle itself, with no dependency on a concrete board — which is what
 * lets it be host-tested. begin(), the one part that has to reach for the
 * board's mutex pool and scheduler, lives in PeriodicTelemetryBackendBoard.cpp.
 */
#include "src/telemetry/PeriodicTelemetryBackend.h"

#include <algorithm>
#include <mutex>

#include "src/utils/Logging.h"

PeriodicTelemetryBackend::PeriodicTelemetryBackend(const char* taskName, float frequencyHz,
                                                   std::uint32_t stackBytes)
    : _taskName(taskName)
    // Clamped before the reciprocal: 0 Hz would give +Inf, and the cast to an
    // integer millisecond count is then undefined.
    , _intervalMs(1000.0f / std::clamp(frequencyHz, 0.1f, 200.0f))
    , _stackBytes(stackBytes)
{
}

PeriodicTelemetryBackend::~PeriodicTelemetryBackend()
{
    // Cooperative: this only asks. It does NOT wait for the loop to leave, so a
    // backend with a real lifetime would need a join here. All of them are
    // file-scope singletons, so no destructor runs in practice.
    requestTaskStop();

    // Pool-owned; nothing reclaims entries (ADR-011, no heap after boot).
    _mutex = nullptr;
}

bool PeriodicTelemetryBackend::beginWith(arduflite::hal::Mutex* mutex,
                                         arduflite::hal::Scheduler& scheduler)
{
    if (_beginAttempted)
    {
        LOG_WARN("%s: begin() called more than once — ignoring", _taskName);
        return false;
    }
    _beginAttempted = true;

    if (mutex == nullptr) { return false; }

    // Set before onBegin() so a subclass can publish() from it, and cleared on
    // every failure path below so the object stays coherently unstarted —
    // publish() and snapshot() both key off this pointer.
    _mutex = mutex;

    if (!onBegin())
    {
        _mutex = nullptr;
        return false;
    }

    arduflite::hal::TaskConfig config;
    config.name       = _taskName;
    config.stackBytes = _stackBytes;
    config.priority   = arduflite::hal::Priority::Telemetry;

    auto task = scheduler.spawn(config, &trampoline, this);
    if (!task)
    {
        LOG_ERR("%s: failed to spawn task — backend disabled", _taskName);
        _mutex = nullptr;
        return false;
    }
    _task = task.value();
    return true;
}

void PeriodicTelemetryBackend::trampoline(void* self)
{
    static_cast<PeriodicTelemetryBackend*>(self)->runLoop();
}

void PeriodicTelemetryBackend::requestTaskStop()
{
    if (_task != nullptr) { _task->requestStop(); }
}

bool PeriodicTelemetryBackend::shouldRun() const
{
    // `_task` is assigned only after spawn() RETURNS, and the body may already
    // be running by then — a higher-priority task preempts immediately. A null
    // handle means "no stop possible yet", not a crash.
    return _task == nullptr || !_task->stopRequested();
}

void PeriodicTelemetryBackend::publish(const TelemetryData& telemData)
{
    if (_mutex == nullptr) { return; }

    std::unique_lock lock(*_mutex, kTelemetryLockTimeout);
    if (!lock.owns_lock()) { return; }
    _pendingData = telemData;
}

bool PeriodicTelemetryBackend::snapshot(TelemetryData& out) const
{
    if (_mutex == nullptr) { return false; }

    std::unique_lock lock(*_mutex, kTelemetryLockTimeout);
    if (!lock.owns_lock()) { return false; }
    out = _pendingData;
    return true;
}
