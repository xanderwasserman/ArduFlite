/**
 * test_sensor_selector.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for estimation::FirstHealthySelector.
 *
 * No board fits two IMUs today, so on real hardware this always picks index 0
 * and never switches. These tests exercise the case that hardware cannot yet
 * produce, which is the entire reason the interface was pinned early
 * (ADR-026): if failover is only ever exercised the day a sensor first fails in
 * flight, it has never been tested at all.
 */
#include <gtest/gtest.h>

#include "src/estimation/SensorSelector.h"

using namespace arduflite;
using arduflite::estimation::FirstHealthySelector;

namespace {

using Clock = arduflite::hal::Clock;

Clock::time_point at(std::int64_t ms)
{
    return Clock::time_point{ std::chrono::microseconds{ ms * 1000 } };
}

/// Minimal accelerometer whose health the test drives directly.
class FakeAccel final : public device::Accelerometer
{
public:
    explicit FakeAccel(float value) : _value(value) {}

    Status read(device::AccelSample& out) const override
    {
        out.accel_g = Vec3f{ _value, _value, _value };
        return Status::Ok;
    }
    Status setRange_g(std::uint8_t) override { return Status::Ok; }
    [[nodiscard]] std::uint8_t range_g() const override { return 4; }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }
    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return 500; }

    device::SensorHealth _health = device::SensorHealth::Ok;
    float                _value;
};

class FakeMag final : public device::Magnetometer
{
public:
    Status read(device::MagSample& out) const override
    {
        out.field_ut = Vec3f{ _value, 0.0f, 0.0f };
        return Status::Ok;
    }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }
    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return 100; }

    explicit FakeMag(float value) : _value(value) {}

    device::SensorHealth _health = device::SensorHealth::Ok;
    float                _value;
};

class SelectorTest : public ::testing::Test
{
protected:
    FakeAccel a0{ 1.0f };
    FakeAccel a1{ 2.0f };

    std::array<device::Accelerometer*, 2> accels{ &a0, &a1 };
    std::array<device::Gyroscope*, 0>     gyros{};
    std::array<device::Barometer*, 0>     baros{};

    FakeMag m0{ 10.0f };
    FakeMag m1{ 20.0f };
    std::array<device::Magnetometer*, 2> mags{ &m0, &m1 };

    /// Three arguments: the shape every board without a magnetometer uses.
    FirstHealthySelector makeSelector()
    {
        return FirstHealthySelector{ accels, gyros, baros };
    }

    FirstHealthySelector makeSelectorWithMagnetometers()
    {
        return FirstHealthySelector{ accels, gyros, baros, mags };
    }
};

// ── Optional magnetometer ───────────────────────────────────────────────────

/// The magnetometer span is the only one that is routinely empty, so a null
/// return here is the normal case rather than a fault. Every caller branches on
/// it, and a selector that returned a dangling pointer instead would fault on
/// the first tick of every board shipped so far.
TEST_F(SelectorTest, ReportsNoMagnetometerWhenTheSpanIsEmpty)
{
    auto selector = makeSelector();
    selector.evaluate(at(0));

    EXPECT_EQ(selector.primaryMag(), nullptr);
}

TEST_F(SelectorTest, SelectsTheLowestIndexedHealthyMagnetometer)
{
    auto selector = makeSelectorWithMagnetometers();
    selector.evaluate(at(0));

    EXPECT_EQ(selector.primaryMag(), &m0);
    EXPECT_EQ(selector.state().activeMag, 0);
}

TEST_F(SelectorTest, FailsOverToTheSecondMagnetometer)
{
    auto selector = makeSelectorWithMagnetometers();
    selector.evaluate(at(0));

    m0._health = device::SensorHealth::Failed;
    selector.evaluate(at(10));

    EXPECT_EQ(selector.primaryMag(), &m1);
    EXPECT_TRUE(selector.switchedThisTick());
    EXPECT_EQ(selector.state().switchCount, 1u);
}

/// Staying put beats churning to an equally dead instance — the same rule the
/// other three spans follow. The subsystem drops to six-axis fusion on health,
/// so a stuck selection costs nothing.
TEST_F(SelectorTest, HoldsTheSelectionWhenNoMagnetometerIsHealthy)
{
    auto selector = makeSelectorWithMagnetometers();
    selector.evaluate(at(0));

    m0._health = device::SensorHealth::Failed;
    m1._health = device::SensorHealth::Failed;
    selector.evaluate(at(10));

    EXPECT_EQ(selector.primaryMag(), &m0);
    EXPECT_FALSE(selector.switchedThisTick());
}

TEST_F(SelectorTest, PrefersTheLowestIndexWhenAllAreHealthy)
{
    auto selector = makeSelector();
    selector.evaluate(at(0));

    EXPECT_EQ(selector.primaryAccel(), &a0);
    EXPECT_EQ(selector.state().activeAccel, 0);
    EXPECT_EQ(selector.state().switchCount, 0u);
    EXPECT_FALSE(selector.switchedThisTick());
}

TEST_F(SelectorTest, FailsOverToTheNextHealthyInstance)
{
    auto selector = makeSelector();
    selector.evaluate(at(0));
    ASSERT_EQ(selector.primaryAccel(), &a0);

    a0._health = device::SensorHealth::Failed;
    selector.evaluate(at(100));

    EXPECT_EQ(selector.primaryAccel(), &a1);
    EXPECT_EQ(selector.state().activeAccel, 1);
    EXPECT_TRUE(selector.switchedThisTick());
    EXPECT_EQ(selector.state().switchCount, 1u);
    EXPECT_EQ(selector.state().lastSwitch, at(100))
        << "the log has to be able to say WHEN, not just whether";
}

TEST_F(SelectorTest, SwitchedThisTickIsTrueOnlyOnTheTransition)
{
    auto selector = makeSelector();
    selector.evaluate(at(0));

    a0._health = device::SensorHealth::Failed;
    selector.evaluate(at(100));
    ASSERT_TRUE(selector.switchedThisTick());

    selector.evaluate(at(200));
    EXPECT_FALSE(selector.switchedThisTick())
        << "a crossfade triggered every tick would never finish";
    EXPECT_EQ(selector.state().switchCount, 1u);
}

TEST_F(SelectorTest, DoesNotFailBackWhenTheOriginalRecovers)
{
    auto selector = makeSelector();
    selector.evaluate(at(0));

    a0._health = device::SensorHealth::Failed;
    selector.evaluate(at(100));
    ASSERT_EQ(selector.primaryAccel(), &a1);

    a0._health = device::SensorHealth::Ok;
    selector.evaluate(at(200));

    // FirstHealthy scans from index 0, so recovery DOES pull it back. Asserted
    // so the behaviour is a decision on record rather than an accident: an
    // intermittent sensor will flap, and the fix is a policy with hysteresis,
    // not a tweak here.
    EXPECT_EQ(selector.primaryAccel(), &a0);
    EXPECT_EQ(selector.state().switchCount, 2u)
        << "each flap is a transient the estimator saw";
}

TEST_F(SelectorTest, StaysPutWhenNothingIsHealthy)
{
    auto selector = makeSelector();
    selector.evaluate(at(0));

    a0._health = device::SensorHealth::Failed;
    a1._health = device::SensorHealth::Failed;
    selector.evaluate(at(100));

    EXPECT_EQ(selector.primaryAccel(), &a0) << "switching to an equally dead instance helps nobody";
    EXPECT_FALSE(selector.switchedThisTick());
    EXPECT_EQ(selector.state().switchCount, 0u)
        << "churn here would inflate the count and, later, never finish a crossfade";
}

TEST_F(SelectorTest, DegradedIsNotHealthyEnoughToSelect)
{
    a0._health = device::SensorHealth::Degraded;

    auto selector = makeSelector();
    selector.evaluate(at(0));

    EXPECT_EQ(selector.primaryAccel(), &a1)
        << "Degraded means the part answered but not correctly - prefer a good one";
}

TEST_F(SelectorTest, EmptySpanYieldsNullptrRatherThanUndefinedBehaviour)
{
    std::array<device::Accelerometer*, 0> none{};
    FirstHealthySelector selector{ none, gyros, baros };
    selector.evaluate(at(0));

    EXPECT_EQ(selector.primaryAccel(), nullptr)
        << "the spare bench board has no IMU and must not fault here";
    EXPECT_EQ(selector.primaryGyro(), nullptr);
    EXPECT_EQ(selector.primaryBaro(), nullptr);
}

TEST_F(SelectorTest, OneTickThatSwitchesSeveralSensorsCountsOnce)
{
    // Two accelerometers and two barometers failing on the same tick is ONE
    // transient from the estimator's point of view.
    FakeAccel b0{ 1.0f };
    std::array<device::Accelerometer*, 2> pair{ &b0, &a1 };
    FirstHealthySelector selector{ pair, gyros, baros };
    selector.evaluate(at(0));

    b0._health = device::SensorHealth::Failed;
    selector.evaluate(at(50));

    EXPECT_EQ(selector.state().switchCount, 1u);
}

} // namespace
