/**
 * test_hal_platform.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * Tests the Tier 0 CONTRACTS against the host implementations. The point is that
 * these contracts are now testable at all: the ESP32 versions must satisfy the
 * same interface, so a contract violation shows up here rather than in flight.
 */
#include <gtest/gtest.h>

#include <chrono>
#include <mutex>
#include <thread>

#include "hal_host/HostPlatform.h"
#include "src/hal/platform/Scheduler.h"

using namespace std::chrono_literals;
using arduflite::Status;
using arduflite::hal::Clock;
using arduflite::hal::toSeconds;
using arduflite::hal::host::HostMutex;
using arduflite::hal::host::HostRecursiveMutex;
using arduflite::hal::host::RecordingPwmOut;
using arduflite::hal::host::VirtualClock;

// ── Clock ───────────────────────────────────────────────────────────────────

TEST(HalClock, AdvancesOnlyWhenTold)
{
    VirtualClock c;
    const auto t0 = c.now();
    c.advanceMs(250);
    EXPECT_EQ(c.now() - t0, 250ms);
}

TEST(HalClock, SurvivesThe32BitMicrosWrap)
{
    // A 32-bit microsecond counter wraps every ~71 minutes; the subtraction
    // in ArduFliteIMU::imuTask would produce a garbage dt at that point.
    VirtualClock c;
    c.setUs(4'294'967'295LL);              // one microsecond short of 2^32
    const auto before = c.now();
    c.advance(Clock::duration{ 2000 });
    const auto dt = c.now() - before;

    EXPECT_EQ(dt, Clock::duration{ 2000 }) << "64-bit chrono must not wrap here";
    EXPECT_NEAR(toSeconds(dt), 0.002f, 1e-9f);
}

TEST(HalClock, ToSecondsMatchesTheControlLoopPeriod)
{
    EXPECT_NEAR(toSeconds(2ms),   0.002f, 1e-9f);   // inner loop
    EXPECT_NEAR(toSeconds(10ms),  0.010f, 1e-9f);   // outer loop
    EXPECT_NEAR(toSeconds(20ms),  0.020f, 1e-9f);   // baro
}

// ── Mutex: the standard lock types must work directly ──────────────────────

TEST(HalMutex, WorksWithStdLockGuard)
{
    HostMutex m;
    {
        std::lock_guard guard(m);
        EXPECT_FALSE(m.try_lock()) << "should already be held";
    }
    EXPECT_TRUE(m.try_lock());
    m.unlock();
}

TEST(HalMutex, WorksWithStdUniqueLockAndTimeout)
{
    HostMutex m;
    std::unique_lock held(m);

    // The bounded-wait idiom every control-loop call site uses.
    std::thread t([&] {
        std::unique_lock attempt(m, 5ms);
        EXPECT_FALSE(attempt) << "must time out while the other lock is held";
    });
    t.join();
}

TEST(HalMutex, ScopedLockTakesTwoWithoutDeadlock)
{
    // std::scoped_lock orders multiple acquisitions internally, so two threads
    // taking the same pair in opposite orders cannot deadlock.
    HostMutex a, b;
    std::thread t1([&] { for (int i = 0; i < 200; ++i) { std::scoped_lock l(a, b); } });
    std::thread t2([&] { for (int i = 0; i < 200; ++i) { std::scoped_lock l(b, a); } });
    t1.join();
    t2.join();
    SUCCEED() << "no deadlock";
}

TEST(HalMutex, RecursiveBusLockCanNest)
{
    // RegisterDevice::busLock() MUST be recursive: a driver grouping several
    // transactions holds it while each transaction also locks internally.
    // With a plain mutex that is an immediate self-deadlock.
    HostRecursiveMutex m;
    std::unique_lock outer(m);
    EXPECT_TRUE(m.try_lock()) << "bus lock must be recursive";
    m.unlock();
}

// ── PwmOut ──────────────────────────────────────────────────────────────────

TEST(HalPwmOut, ClampsToConfiguredEndpoints)
{
    RecordingPwmOut p;
    ASSERT_EQ(p.attach(1000, 2000, 50), Status::Ok);

    p.writeMicroseconds(500);
    EXPECT_EQ(p.lastMicroseconds(), 1000);

    p.writeMicroseconds(3000);
    EXPECT_EQ(p.lastMicroseconds(), 2000);

    p.writeMicroseconds(1500);
    EXPECT_EQ(p.lastMicroseconds(), 1500);
}

TEST(HalPwmOut, RejectsInvertedEndpoints)
{
    RecordingPwmOut p;
    EXPECT_EQ(p.attach(2000, 1000, 50), Status::InvalidArg);
}

TEST(HalPwmOut, IgnoresWritesBeforeAttach)
{
    RecordingPwmOut p;
    p.writeMicroseconds(1500);
    EXPECT_EQ(p.lastMicroseconds(), 0);
    EXPECT_TRUE(p.writes.empty());
}

TEST(HalPwmOut, IdleStopsPulsingWithoutDetaching)
{
    // FailsafeAction::Release means no pulse at all, which is different from
    // detaching the channel.
    RecordingPwmOut p;
    ASSERT_EQ(p.attach(1000, 2000, 50), Status::Ok);
    p.writeMicroseconds(1500);
    p.idle();

    EXPECT_TRUE(p.isIdle());
    EXPECT_TRUE(p.isAttached());
    EXPECT_EQ(p.lastMicroseconds(), 0);
}

TEST(HalPwmOut, RecordsEveryWriteWithATimestamp)
{
    VirtualClock c;
    RecordingPwmOut p(&c);
    ASSERT_EQ(p.attach(1000, 2000, 50), Status::Ok);

    p.writeMicroseconds(1500);
    c.advanceMs(2);
    p.writeMicroseconds(1600);

    ASSERT_EQ(p.writes.size(), 2u);
    EXPECT_EQ(p.writes[0].us, 1500);
    EXPECT_EQ(p.writes[1].us, 1600);
    EXPECT_EQ(p.writes[1].at - p.writes[0].at, 2ms)
        << "this is how Phase 3 will assert slew-rate limiting";
}

// ── Microsecond duty maths, mirroring Esp32PwmOut ──────────────────────────

TEST(HalPwmOut, MicrosecondToDutyMathsIsExactAtTheEndpoints)
{
    // Esp32PwmOut converts us -> LEDC duty with integer maths (no FPU on the C3).
    // Reproduced here so the arithmetic is pinned; the ESP32 version must match.
    constexpr std::uint8_t  kBits    = 14;
    constexpr std::uint16_t kFrameHz = 50;
    const std::uint32_t periodUs = 1000000u / kFrameHz;      // 20000
    const std::uint32_t maxDuty  = (1u << kBits) - 1u;       // 16383

    auto duty = [&](std::uint32_t us) { return (us * maxDuty) / periodUs; };

    EXPECT_EQ(duty(1000), 819u);
    EXPECT_EQ(duty(1500), 1228u);
    EXPECT_EQ(duty(2000), 1638u);

    // Resolution must be better than 1 us, or slew limiting is meaningless.
    EXPECT_GT(duty(1501), duty(1500)) << "1 us must be distinguishable";
}

// ── Priority ladder ─────────────────────────────────────────────────────────
//
// These values are measured from the firmware's xTaskCreate calls. An earlier
// version of the enum was written from AGENTS.md's prose ladder and had RcLink
// at 3 (the CRSF parser actually runs at 2) and Cli at 0 (actually 1). Migrating
// a task to a mismatched value silently changes scheduling — on the RC path,
// with no flight testing to catch it.

TEST(HalPriority, MatchesTheMeasuredFirmwareValues)
{
    using arduflite::hal::Priority;

    EXPECT_EQ(static_cast<int>(Priority::Cli),       1);
    EXPECT_EQ(static_cast<int>(Priority::Web),       1);
    EXPECT_EQ(static_cast<int>(Priority::Config),    1);
    EXPECT_EQ(static_cast<int>(Priority::Telemetry), 1);
    EXPECT_EQ(static_cast<int>(Priority::Mission),   1);
    EXPECT_EQ(static_cast<int>(Priority::Indicator), 1);
    EXPECT_EQ(static_cast<int>(Priority::OuterLoop), 2);
    EXPECT_EQ(static_cast<int>(Priority::RcLink),    2);
    EXPECT_EQ(static_cast<int>(Priority::InnerLoop), 3);
    EXPECT_EQ(static_cast<int>(Priority::Inertial),  4);
}

TEST(HalPriority, PreservesTheOrderingThatMattersForFlight)
{
    using arduflite::hal::Priority;

    // Sensing must preempt inner control; inner must preempt outer; both must
    // preempt anything in the background band.
    EXPECT_GT(static_cast<int>(Priority::Inertial),  static_cast<int>(Priority::InnerLoop));
    EXPECT_GT(static_cast<int>(Priority::InnerLoop), static_cast<int>(Priority::OuterLoop));
    EXPECT_GT(static_cast<int>(Priority::OuterLoop), static_cast<int>(Priority::Telemetry));
    EXPECT_GT(static_cast<int>(Priority::OuterLoop), static_cast<int>(Priority::Cli));

    // RC input shares the outer-loop band — it must NOT be level with the inner
    // loop, which is what the incorrect RcLink = 3 would have produced.
    EXPECT_EQ(static_cast<int>(Priority::RcLink), static_cast<int>(Priority::OuterLoop));
    EXPECT_LT(static_cast<int>(Priority::RcLink), static_cast<int>(Priority::InnerLoop));
}


// ── Task stop contract ──────────────────────────────────────────────────────
using arduflite::hal::TaskConfig;
using arduflite::hal::host::RecordingScheduler;
using arduflite::Status;

//
// Stopping is cooperative: requestStop() asks, stopRequested() is how the body
// hears, and isRunning() reports whether the body has actually left. All three
// have to work together, or a stop request is a no-op that looks like a stop
// request. ADR-058.

namespace {

/// A body that polls the stop flag, with an escape hatch so a broken flag
/// cannot hang the suite. The two tests below pin both outcomes, so the escape
/// hatch cannot be what makes either of them pass.
struct StopPollingBody
{
    arduflite::hal::Task* task       = nullptr;
    int                   iterations = 0;

    static void run(void* arg)
    {
        auto* self = static_cast<StopPollingBody*>(arg);
        while (!self->task->stopRequested() && self->iterations < 1000)
        {
            ++self->iterations;
        }
    }
};

} // namespace

TEST(TaskStopContract, ATaskStartsRunningAndNotStopped)
{
    RecordingScheduler scheduler;
    TaskConfig config;
    config.name = "probe";

    auto task = scheduler.spawn(config, [](void*) {}, nullptr);
    ASSERT_TRUE(task);

    EXPECT_TRUE(task.value()->isRunning());
    EXPECT_FALSE(task.value()->stopRequested());
}

/// The property that was missing: what requestStop() sets, the body can read.
TEST(TaskStopContract, RequestStopIsVisibleToTheTaskBody)
{
    RecordingScheduler scheduler;
    TaskConfig config;
    config.name = "probe";

    auto task = scheduler.spawn(config, [](void*) {}, nullptr);
    ASSERT_TRUE(task);

    task.value()->requestStop();
    EXPECT_TRUE(task.value()->stopRequested())
        << "the stop request has to be observable, or it is not a stop request";
}

/// A body that polls the flag must be able to leave, and isRunning() must then
/// report false rather than "was spawned once".
TEST(TaskStopContract, ABodyThatHonoursTheFlagLeavesAndStopsRunning)
{
    RecordingScheduler scheduler;
    TaskConfig config;
    config.name = "probe";

    StopPollingBody body;
    auto task = scheduler.spawn(config, &StopPollingBody::run, &body);
    ASSERT_TRUE(task);
    body.task = task.value();

    // Stop before the body is driven, so the loop exits on its first check
    // rather than on the iteration limit.
    task.value()->requestStop();
    scheduler.lastTask.runBody();

    EXPECT_EQ(body.iterations, 0) << "the body did not check the flag first";
    EXPECT_FALSE(task.value()->isRunning());
}

/// Proves the iteration limit is not what ended the loop above.
TEST(TaskStopContract, ABodyRunsOnWhenNoStopWasRequested)
{
    RecordingScheduler scheduler;
    TaskConfig config;
    config.name = "probe";

    StopPollingBody body;
    auto task = scheduler.spawn(config, &StopPollingBody::run, &body);
    ASSERT_TRUE(task);
    body.task = task.value();

    scheduler.lastTask.runBody();
    EXPECT_EQ(body.iterations, 1000) << "no stop was asked for, so it should run on";
}

TEST(TaskStopContract, SpawnRejectsANullBody)
{
    RecordingScheduler scheduler;
    TaskConfig config;
    config.name = "probe";

    auto task = scheduler.spawn(config, nullptr, nullptr);
    EXPECT_FALSE(task);
    EXPECT_EQ(task.status(), Status::InvalidArg);
}
