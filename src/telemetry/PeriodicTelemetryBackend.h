/**
 * PeriodicTelemetryBackend.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Task and mutex lifecycle shared by every periodic telemetry backend.
 *
 * A backend supplies its own loop body and nothing else: this owns the mutex
 * that guards the published snapshot, the task that runs the loop, and the
 * ordering rules between them.
 *
 * Those rules are easy to get subtly wrong and hard to notice when you do. The
 * task may start running before spawn() returns, so a stop check that
 * dereferences the task handle can fault on the first iteration. The snapshot
 * lock is a bounded WAIT, not a try-lock, because publish() and the loop both
 * run at telemetry rate and brief overlap is normal — giving up instantly drops
 * samples. Both live here once rather than in each backend.
 */
#ifndef ARDUFLITE_TELEMETRY_PERIODIC_BACKEND_H
#define ARDUFLITE_TELEMETRY_PERIODIC_BACKEND_H

#include <cstdint>

#include "src/hal/platform/Mutex.h"
#include "src/hal/platform/Scheduler.h"
#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"

class PeriodicTelemetryBackend : public ArduFliteTelemetry
{
public:
    /**
     * @brief Store a snapshot for the loop to pick up. Safe from any task.
     *
     * Drops the sample if the lock is not free within kTelemetryLockTimeout,
     * so a caller in the flight path is never blocked by a slow backend.
     */
    void publish(const TelemetryData& telemData) final;

    /// Allocate the mutex, run onBegin(), spawn the task. Idempotent.
    void begin() final;

    ~PeriodicTelemetryBackend() override;

protected:
    /**
     * @param taskName    FreeRTOS task name; must outlive the task.
     * @param frequencyHz loop rate, clamped to [0.1, 200].
     * @param stackBytes  task stack. Raise it for a backend that formats into
     *                    large buffers on the stack.
     */
    PeriodicTelemetryBackend(const char* taskName, float frequencyHz,
                             std::uint32_t stackBytes = 4096);

    /**
     * @brief Backend-specific setup, run after the mutex exists and BEFORE the
     *        task is spawned.
     * @return false to abort begin(); the object stays coherently unstarted.
     */
    virtual bool onBegin() { return true; }

    /// The backend's loop. Runs until shouldRun() reports false, then returns.
    virtual void runLoop() = 0;

    /// Loop condition. A null task means "started, no stop possible yet" —
    /// spawn() may not have returned when the body first runs.
    [[nodiscard]] bool shouldRun() const;

    /**
     * @brief Copy the last published sample.
     * @return false on lock timeout, leaving @p out UNTOUCHED so the caller
     *         keeps whatever it had. A backend that must not repeat a sample
     *         checks this and skips the iteration instead.
     */
    [[nodiscard]] bool snapshot(TelemetryData& out) const;

    /**
     * @brief Ask the loop to stop, from a derived destructor.
     *
     * Destructors run derived-first, so a backend that has to order its own
     * teardown against the loop — closing a file, flushing a buffer — must ask
     * before it starts, not rely on ~PeriodicTelemetryBackend() doing it after.
     *
     * Cooperative, so it does not wait. Order against work already in progress
     * by taking whatever lock that work holds.
     */
    void requestTaskStop();

    /// The task's name, for diagnostics.
    [[nodiscard]] const char* taskName() const { return _taskName; }

    /// Milliseconds between iterations, derived from the constructor's Hz.
    [[nodiscard]] float intervalMs() const { return _intervalMs; }

    /**
     * @brief Start the lifecycle against explicitly supplied platform services.
     *
     * begin() forwards to this with the board's. Exposed so host tests can
     * drive the whole lifecycle without a board.
     */
    bool beginWith(arduflite::hal::Mutex* mutex, arduflite::hal::Scheduler& scheduler);

private:
    static void trampoline(void* self);

    /// begin() is a one-shot. A backend that could not take a mutex or spawn a
    /// task at boot will not manage it later — the mutex pool does not grow and
    /// scheduler slots are not reclaimed (ADR-011) — and RETRYING leaks: both
    /// this class and onBegin() would allocate a second time.
    bool _beginAttempted = false;

    const char*   _taskName;
    float         _intervalMs;
    std::uint32_t _stackBytes;

    arduflite::hal::Task*  _task  = nullptr;   ///< Owned by the scheduler
    arduflite::hal::Mutex* _mutex = nullptr;   ///< Guards _pendingData
    TelemetryData          _pendingData{};
};

#endif // ARDUFLITE_TELEMETRY_PERIODIC_BACKEND_H
