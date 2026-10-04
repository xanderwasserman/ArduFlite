/**
 * test_periodic_backend.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the telemetry backends' shared lifecycle.
 *
 * The rules under test are the ones that were previously copied into each
 * backend by hand, and which diverged when they were: the task guard that must
 * tolerate a null handle, the snapshot lock being a bounded WAIT rather than a
 * try-lock, and every failure path leaving the object coherently unstarted.
 */
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "hal_host/HostPlatform.h"
#include "src/telemetry/PeriodicTelemetryBackend.h"

using arduflite::hal::host::HostMutex;
using arduflite::hal::host::RecordingScheduler;

/**
 * begin() reaches for the board's mutex pool and scheduler, and lives in
 * PeriodicTelemetryBackendBoard.cpp — which cannot link on a host. The vtable
 * still needs a definition, so here is one.
 *
 * It fails rather than doing nothing: these tests drive beginWith() with
 * explicit platform services, and a test that reached for begin() would
 * otherwise silently start nothing and then assert on the result.
 */
void PeriodicTelemetryBackend::begin()
{
    ADD_FAILURE() << "begin() needs a board; host tests must call beginWith()";
}

namespace {

/// Minimal backend: records what the lifecycle handed it.
class FakeBackend final : public PeriodicTelemetryBackend
{
public:
    FakeBackend() : PeriodicTelemetryBackend("FakeTel", 50.0f) {}

    bool onBegin() override { ++onBeginCalls; return onBeginResult; }

    void runLoop() override
    {
        while (shouldRun() && iterations < 1000)
        {
            ++iterations;
            TelemetryData local{};
            if (snapshot(local)) { ++freshCount; lastSeen = local; }
            else                 { ++staleCount; }
        }
    }

    // Exposed so the tests can drive the lifecycle directly.
    using PeriodicTelemetryBackend::beginWith;
    using PeriodicTelemetryBackend::intervalMs;
    using PeriodicTelemetryBackend::snapshot;

    int  onBeginCalls  = 0;
    bool onBeginResult = true;
    int  iterations    = 0;
    int  freshCount    = 0;
    int  staleCount    = 0;
    TelemetryData lastSeen{};
};

TelemetryData sampleWithAltitude(float altitude)
{
    TelemetryData d{};
    d.altitude = altitude;
    return d;
}

/**
 * @brief Holds a mutex from ANOTHER thread for a given duration.
 *
 * Locking it on the test's own thread and then calling into the backend would
 * be a recursive acquire of a std::timed_mutex, which is undefined behaviour —
 * it happens to return false on this platform, so such a test looks like it
 * works while resting on nothing.
 */
class LockHolder
{
public:
    LockHolder(arduflite::hal::Mutex& mutex, std::chrono::milliseconds hold)
    {
        _thread = std::thread([&mutex, hold, this] {
            mutex.lock();
            _held.store(true);
            std::this_thread::sleep_for(hold);
            mutex.unlock();
        });
        while (!_held.load()) { std::this_thread::yield(); }
    }
    ~LockHolder() { _thread.join(); }

private:
    std::thread       _thread;
    std::atomic<bool> _held{ false };
};

} // namespace

// ── Lifecycle ───────────────────────────────────────────────────────────────

TEST(PeriodicBackend, BeginAllocatesAndSpawnsOnce)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;

    EXPECT_TRUE(backend.beginWith(&mutex, scheduler));
    EXPECT_EQ(backend.onBeginCalls, 1);
    ASSERT_EQ(scheduler.spawned.size(), 1u);
    EXPECT_STREQ(scheduler.spawned[0], "FakeTel");
}

/// A second begin() must not spawn a second task racing the first on the same
/// pending sample.
TEST(PeriodicBackend, BeginIsIdempotent)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;

    ASSERT_TRUE(backend.beginWith(&mutex, scheduler));
    EXPECT_FALSE(backend.beginWith(&mutex, scheduler));

    EXPECT_EQ(backend.onBeginCalls, 1);
    EXPECT_EQ(scheduler.spawned.size(), 1u);
}

/// onBegin() is the backend's own setup — if it fails there is nothing to run.
TEST(PeriodicBackend, AFailedOnBeginAbortsBeforeSpawning)
{
    FakeBackend backend;
    backend.onBeginResult = false;
    HostMutex mutex;
    RecordingScheduler scheduler;

    EXPECT_FALSE(backend.beginWith(&mutex, scheduler));
    EXPECT_TRUE(scheduler.spawned.empty());
}

/// Coherently unstarted, not half-started: publish() keys off the same mutex
/// pointer, so a failure that left it set would accept samples nothing reads.
TEST(PeriodicBackend, AFailedOnBeginLeavesPublishInert)
{
    FakeBackend backend;
    backend.onBeginResult = false;
    HostMutex mutex;
    RecordingScheduler scheduler;

    ASSERT_FALSE(backend.beginWith(&mutex, scheduler));

    backend.publish(sampleWithAltitude(42.0f));
    TelemetryData out{};
    EXPECT_FALSE(backend.snapshot(out)) << "no mutex, so nothing to read";
}

TEST(PeriodicBackend, AFailedSpawnAlsoLeavesPublishInert)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;
    scheduler.spawnFails = true;

    EXPECT_FALSE(backend.beginWith(&mutex, scheduler));

    backend.publish(sampleWithAltitude(7.0f));
    TelemetryData out{};
    EXPECT_FALSE(backend.snapshot(out));
}

/**
 * begin() is one-shot even when it FAILS.
 *
 * Retrying would allocate a second mutex here and a second one inside
 * onBegin(), from a 16-entry pool that never reclaims — so a retry loop would
 * quietly exhaust it. And there is nothing to gain: a backend that could not
 * take a mutex or spawn a task at boot will not manage it later, because
 * neither resource is ever freed.
 */
TEST(PeriodicBackend, BeginDoesNotRetryAfterAFailure)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;

    scheduler.spawnFails = true;
    ASSERT_FALSE(backend.beginWith(&mutex, scheduler));
    EXPECT_EQ(backend.onBeginCalls, 1);

    scheduler.spawnFails = false;
    EXPECT_FALSE(backend.beginWith(&mutex, scheduler))
        << "a second attempt must be refused, not allocate again";
    EXPECT_EQ(backend.onBeginCalls, 1) << "onBegin must not run twice";
    EXPECT_TRUE(scheduler.spawned.empty());
}

// ── publish / snapshot ──────────────────────────────────────────────────────

TEST(PeriodicBackend, PublishIsVisibleToSnapshot)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;
    ASSERT_TRUE(backend.beginWith(&mutex, scheduler));

    backend.publish(sampleWithAltitude(123.5f));

    TelemetryData out{};
    ASSERT_TRUE(backend.snapshot(out));
    EXPECT_FLOAT_EQ(out.altitude, 123.5f);
}

TEST(PeriodicBackend, PublishBeforeBeginIsDroppedRatherThanCrashing)
{
    FakeBackend backend;
    backend.publish(sampleWithAltitude(1.0f));   // no mutex yet

    TelemetryData out{};
    EXPECT_FALSE(backend.snapshot(out));
}

/**
 * The behaviour chosen in Phase 10: on a lock timeout `out` is left ALONE and
 * false is returned, so a caller holding a previous copy keeps it. Overwriting
 * it with a default-constructed sample would put zeroed attitude into a log.
 */
TEST(PeriodicBackend, SnapshotLeavesTheDestinationUntouchedOnTimeout)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;
    ASSERT_TRUE(backend.beginWith(&mutex, scheduler));

    backend.publish(sampleWithAltitude(500.0f));

    TelemetryData out = sampleWithAltitude(99.0f);
    {
        LockHolder holder(mutex, kTelemetryLockTimeout * 4);
        EXPECT_FALSE(backend.snapshot(out));
    }

    EXPECT_FLOAT_EQ(out.altitude, 99.0f) << "the caller's previous copy survives";
}

/// A bounded WAIT, not a try-lock: publish() and the loop both run at telemetry
/// rate, so brief overlap is routine and giving up instantly drops samples.
TEST(PeriodicBackend, SnapshotWaitsRatherThanFailingImmediately)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;
    ASSERT_TRUE(backend.beginWith(&mutex, scheduler));
    backend.publish(sampleWithAltitude(3.0f));

    LockHolder holder(mutex, kTelemetryLockTimeout * 4);

    const auto start = std::chrono::steady_clock::now();
    TelemetryData out{};
    EXPECT_FALSE(backend.snapshot(out));
    const auto waited = std::chrono::steady_clock::now() - start;

    EXPECT_GE(waited, kTelemetryLockTimeout / 2)
        << "returned too fast to have waited — this is a try-lock, not a wait";
}

// ── Loop control ────────────────────────────────────────────────────────────

TEST(PeriodicBackend, TheLoopRunsUntilAStopIsRequested)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;
    ASSERT_TRUE(backend.beginWith(&mutex, scheduler));

    scheduler.lastTask.requestStop();
    scheduler.lastTask.runBody();

    EXPECT_EQ(backend.iterations, 0) << "the guard is checked before the body";
    EXPECT_FALSE(scheduler.lastTask.isRunning());
}

/**
 * The body may run BEFORE spawn() returns — a higher-priority task preempts
 * immediately — so the loop guard has to tolerate a task handle that has not
 * been assigned yet. Running runLoop() before beginWith() is the only way to
 * reproduce that ordering on a host, where spawn() cannot preempt.
 */
TEST(PeriodicBackend, TheLoopGuardToleratesAnUnassignedTaskHandle)
{
    FakeBackend backend;
    backend.runLoop();   // no begin() at all: _task and _mutex are both null

    EXPECT_EQ(backend.iterations, 1000) << "a null handle must mean keep going";
    EXPECT_EQ(backend.staleCount, 1000) << "and no mutex means no snapshot";
}

TEST(PeriodicBackend, TheLoopRunsWhenNoStopWasRequested)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;
    ASSERT_TRUE(backend.beginWith(&mutex, scheduler));

    scheduler.lastTask.runBody();
    EXPECT_EQ(backend.iterations, 1000) << "ran to its own limit, not stopped";
}

/// Every iteration got a real sample, so the stale path is not what drove the
/// counts above.
TEST(PeriodicBackend, TheLoopSeesPublishedData)
{
    FakeBackend backend;
    HostMutex mutex;
    RecordingScheduler scheduler;
    ASSERT_TRUE(backend.beginWith(&mutex, scheduler));

    backend.publish(sampleWithAltitude(77.0f));
    scheduler.lastTask.runBody();

    EXPECT_EQ(backend.staleCount, 0);
    EXPECT_EQ(backend.freshCount, backend.iterations);
    EXPECT_FLOAT_EQ(backend.lastSeen.altitude, 77.0f);
}

// ── Rate ────────────────────────────────────────────────────────────────────

TEST(PeriodicBackend, FrequencyBecomesAnInterval)
{
    FakeBackend backend;                       // 50 Hz
    EXPECT_FLOAT_EQ(backend.intervalMs(), 20.0f);
}

/// Zero would make the reciprocal infinite, and the cast to an integer
/// millisecond count undefined.
TEST(PeriodicBackend, ImplausibleFrequenciesAreClamped)
{
    class FastBackend final : public PeriodicTelemetryBackend {
    public:
        FastBackend(float hz) : PeriodicTelemetryBackend("t", hz) {}
        void runLoop() override {}
        using PeriodicTelemetryBackend::intervalMs;
    };

    EXPECT_FLOAT_EQ(FastBackend(0.0f).intervalMs(),    1000.0f / 0.1f);
    EXPECT_FLOAT_EQ(FastBackend(-5.0f).intervalMs(),   1000.0f / 0.1f);
    EXPECT_FLOAT_EQ(FastBackend(10000.0f).intervalMs(), 1000.0f / 200.0f);
}
