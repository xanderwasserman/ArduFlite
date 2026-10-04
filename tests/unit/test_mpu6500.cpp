/**
 * test_mpu6500.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the MPU-6500 register driver.
 *
 * The point of writing our own driver (ADR-013) was that FastIMU bound to a
 * TwoWire& and so could not be tested anywhere but on hardware. These tests are
 * that claim being cashed in: every register write, every scaling constant and
 * the one-transaction sampling contract are checked with no hardware present.
 */
#include <gtest/gtest.h>

#include "hal_host/HostPlatform.h"
#include "src/hal/drivers/imu/Mpu6500.h"

using namespace arduflite;
using arduflite::hal::host::FakeRegisterDevice;
using arduflite::hal::host::RecordingScheduler;
using arduflite::hal::host::VirtualClock;

namespace {

class Mpu6500Test : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dev.regs[drivers::Mpu6500::kRegWhoAmI] = drivers::Mpu6500::kWhoAmIExpected;
    }

    /// Last value written to reg, or -1 if it was never written.
    int lastWrite(std::uint8_t reg) const
    {
        int found = -1;
        for (const auto& w : dev.writeLog)
        {
            if (w.reg == reg) { found = w.value; }
        }
        return found;
    }

    FakeRegisterDevice dev;
    VirtualClock       clock;
    RecordingScheduler scheduler;
};

// ── Identification ──────────────────────────────────────────────────────────

TEST_F(Mpu6500Test, ProbeAcceptsGenuinePart)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    EXPECT_EQ(imu.probe(), Status::Ok);
    EXPECT_EQ(imu.whoAmI(), 0x70);
    EXPECT_EQ(imu.health(), device::SensorHealth::Ok);
}

/**
 * The reason this driver exists at all. FastIMU did:
 *     if (!(whoami == 0x70)) return -1;
 * with no record of what the part actually said. The maintainer suspects a
 * counterfeit IMU on the prototype; that byte is the diagnosis.
 */
TEST_F(Mpu6500Test, ProbeRecordsTheActualIdOnMismatch)
{
    dev.regs[drivers::Mpu6500::kRegWhoAmI] = 0x71;   // MPU-9250 signature

    drivers::Mpu6500 imu(dev, clock, scheduler);
    EXPECT_EQ(imu.probe(), Status::NotPresent);
    EXPECT_EQ(imu.whoAmI(), 0x71) << "the whole point is that this is recoverable";
    EXPECT_EQ(imu.health(), device::SensorHealth::Degraded)
        << "a wrong ID is a warning, not a dead bus";
}

TEST_F(Mpu6500Test, ProbeReportsNotPresentWhenTheBusIsDead)
{
    dev.failNextRead = 1;

    drivers::Mpu6500 imu(dev, clock, scheduler);
    EXPECT_EQ(imu.probe(), Status::NotPresent);
    EXPECT_EQ(imu.health(), device::SensorHealth::NotPresent)
        << "distinguishable from a responding part with the wrong ID";
}

// ── Configuration ───────────────────────────────────────────────────────────

/**
 * Pins the exact register state FastIMU left behind, given ArduFlite's overrides
 * (setGyroRange(500), setAccelRange(4) after FastIMU's own init). If a future
 * change alters the flying configuration, it fails here rather than in the air.
 */
TEST_F(Mpu6500Test, BeginReproducesTheLegacyRegisterConfiguration)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegPwrMgmt1), 0x01) << "clock = gyro X PLL";
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegPwrMgmt2), 0x00) << "accel + gyro enabled";

    // setGyroODR(333) -> 1000/333 - 1 = 2 -> 1000/(1+2) = 333 Hz
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegSmplrtDiv), 2);

    // gyro LPF 42 Hz -> DLPF_CFG 3
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegConfig) & 0x07, 3);
    // accel LPF 41 Hz -> A_DLPF_CFG 3
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegAccelConfig2) & 0x0F, 3);

    // FCHOICE_B must be cleared or DLPF_CFG is bypassed entirely.
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegGyroConfig) & 0x03, 0);

    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegGyroConfig)  & 0x18, 0x08) << "500 dps";
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegAccelConfig) & 0x18, 0x08) << "4 g";

    EXPECT_EQ(imu.range_dps(), 500);
    EXPECT_EQ(imu.range_g(), 4);
    EXPECT_EQ(imu.nativeRate_hz(), 333);
}

TEST_F(Mpu6500Test, BeginResetsBeforeConfiguring)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    ASSERT_FALSE(dev.writeLog.empty());
    EXPECT_EQ(dev.writeLog[0].reg, drivers::Mpu6500::kRegPwrMgmt1);
    EXPECT_EQ(dev.writeLog[0].value, 0x80)
        << "configuring a device that was not reset inherits whatever the last boot left";
}

/**
 * A soft reset reloads the register file from internal defaults, and writes
 * issued while that is happening are dropped. Skipping the settling delays gives
 * an IMU that comes up partly configured — typically at the wrong range, so
 * every reading is off by a constant factor — intermittently, depending on bus
 * timing and temperature.
 *
 * An earlier revision of this driver had no delays at all: the 100 ms was spent
 * in Board::begin() BEFORE the driver ran, i.e. before the reset it was meant to
 * follow. Nothing caught it, because a fake bus answers instantly and hardware
 * usually gets away with it.
 */
TEST_F(Mpu6500Test, BeginWaitsForTheChipToSettleAfterReset)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    ASSERT_GE(scheduler.sleepLog.size(), 3u) << "reset, wake and PLL lock each need one";
    EXPECT_EQ(scheduler.sleepLog[0], std::chrono::milliseconds(100)) << "after reset";
    EXPECT_EQ(scheduler.sleepLog[1], std::chrono::milliseconds(100)) << "after wake";
    EXPECT_EQ(scheduler.sleepLog[2], std::chrono::milliseconds(200)) << "PLL lock";

    // The first delay must come after the reset write, not before it.
    ASSERT_FALSE(dev.writeLog.empty());
    EXPECT_EQ(dev.writeLog[0].reg, drivers::Mpu6500::kRegPwrMgmt1);
    EXPECT_EQ(dev.writeLog[0].value, 0x80);
}

TEST_F(Mpu6500Test, SamplingNeverSleeps)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    const std::size_t afterBegin = scheduler.sleepLog.size();
    ASSERT_EQ(imu.sample(), Status::Ok);

    EXPECT_EQ(scheduler.sleepLog.size(), afterBegin)
        << "sample() runs in the 500 Hz task; a blocking delay there is fatal";
}

TEST_F(Mpu6500Test, BeginPropagatesBusFailure)
{
    dev.failNextWrite = 1;

    drivers::Mpu6500 imu(dev, clock, scheduler);
    EXPECT_EQ(imu.begin(), Status::IoError) << "must not report a configured device";
}

TEST_F(Mpu6500Test, RangeSettersRejectValuesTheHardwareCannotEncode)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    EXPECT_EQ(imu.setRange_g(3), Status::InvalidArg);
    EXPECT_EQ(imu.setRange_dps(400), Status::InvalidArg);

    // A rejected range must not disturb the configured one.
    EXPECT_EQ(imu.range_g(), 4);
    EXPECT_EQ(imu.range_dps(), 500);
}

TEST_F(Mpu6500Test, RangeSettersPreserveUnrelatedBits)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);
    ASSERT_EQ(imu.setRange_dps(2000), Status::Ok);

    // Range lives in [4:3]; FCHOICE_B in [1:0] must survive a read-modify-write.
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegGyroConfig) & 0x18, 0x18);
    EXPECT_EQ(lastWrite(drivers::Mpu6500::kRegGyroConfig) & 0x03, 0x00);
}

// ── Sampling ────────────────────────────────────────────────────────────────

/**
 * The ADR-019 contract, asserted directly: sample() is one burst, read() is free.
 * Without this split, three per-measurement reads at 500 Hz would triple I2C
 * traffic and let the accel and gyro come from different sensor frames.
 */
TEST_F(Mpu6500Test, SampleIsOneTransactionAndReadIsNone)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    dev.transactions = 0;
    ASSERT_EQ(imu.sample(), Status::Ok);
    EXPECT_EQ(dev.transactions, 1);
    EXPECT_EQ(dev.lastReadLen, 14u) << "accel(6) + temp(2) + gyro(6)";

    device::AccelSample a{};
    device::GyroSample  g{};
    ASSERT_EQ(imu.read(a), Status::Ok);
    ASSERT_EQ(imu.read(g), Status::Ok);
    ASSERT_EQ(imu.read(a), Status::Ok);

    EXPECT_EQ(dev.transactions, 1) << "read() must never touch the bus";
}

TEST_F(Mpu6500Test, ScalingMatchesTheLegacyValues)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    // Full-scale positive on X, half-scale negative on Y, zero on Z.
    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 0, 32767);
    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 2, -16384);
    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 4, 0);
    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 8, 16384);
    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 10, -32768);
    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 12, 0);

    ASSERT_EQ(imu.sample(), Status::Ok);

    device::AccelSample a{};
    ASSERT_EQ(imu.read(a), Status::Ok);
    EXPECT_NEAR(a.accel_g.x,  4.0f * 32767.0f / 32768.0f, 1e-4f);
    EXPECT_NEAR(a.accel_g.y, -2.0f, 1e-4f);
    EXPECT_NEAR(a.accel_g.z,  0.0f, 1e-6f);

    device::GyroSample g{};
    ASSERT_EQ(imu.read(g), Status::Ok);
    EXPECT_NEAR(g.rate_dps.x,  250.0f, 1e-3f);
    EXPECT_NEAR(g.rate_dps.y, -500.0f, 1e-3f);
    EXPECT_NEAR(g.rate_dps.z,    0.0f, 1e-6f);
}

/// Sign handling is the classic register-driver bug: a byte pair read as
/// unsigned turns -1 g into +15.9995 g and the aircraft inverts.
TEST_F(Mpu6500Test, NegativeValuesDecodeAsTwosComplement)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    dev.regs[drivers::Mpu6500::kRegAccelXoutH + 0] = 0xFF;
    dev.regs[drivers::Mpu6500::kRegAccelXoutH + 1] = 0xFF;   // -1

    ASSERT_EQ(imu.sample(), Status::Ok);

    device::AccelSample a{};
    ASSERT_EQ(imu.read(a), Status::Ok);
    EXPECT_LT(a.accel_g.x, 0.0f);
    EXPECT_NEAR(a.accel_g.x, -4.0f / 32768.0f, 1e-9f);
}

TEST_F(Mpu6500Test, RangeChangeRescalesSubsequentSamples)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);
    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH, 16384);

    ASSERT_EQ(imu.sample(), Status::Ok);
    device::AccelSample a{};
    ASSERT_EQ(imu.read(a), Status::Ok);
    EXPECT_NEAR(a.accel_g.x, 2.0f, 1e-4f);

    ASSERT_EQ(imu.setRange_g(16), Status::Ok);
    ASSERT_EQ(imu.sample(), Status::Ok);
    ASSERT_EQ(imu.read(a), Status::Ok);
    EXPECT_NEAR(a.accel_g.x, 8.0f, 1e-4f) << "same raw counts, four times the range";
}

TEST_F(Mpu6500Test, TemperatureUsesTheDatasheetTransfer)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 6, 0);
    ASSERT_EQ(imu.sample(), Status::Ok);
    EXPECT_NEAR(imu.temperature_c(), 21.0f, 1e-3f);

    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH + 6, 3339);
    ASSERT_EQ(imu.sample(), Status::Ok);
    EXPECT_NEAR(imu.temperature_c(), 31.0f, 0.05f);
}

TEST_F(Mpu6500Test, BothSamplesShareOneTimestamp)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    clock.advanceMs(1234);   // move it off zero, or this asserts 0 == 0
    ASSERT_EQ(imu.sample(), Status::Ok);

    device::AccelSample a{};
    device::GyroSample  g{};
    ASSERT_EQ(imu.read(a), Status::Ok);
    ASSERT_EQ(imu.read(g), Status::Ok);

    EXPECT_EQ(a.time, g.time)
        << "they came out of one burst; fusion assumes they are simultaneous";
    EXPECT_NE(a.time.time_since_epoch().count(), 0)
        << "guards against this passing because the clock never moved";
}

TEST_F(Mpu6500Test, FailedSampleIsReportedAndDoesNotFabricateData)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    dev.setBe16(drivers::Mpu6500::kRegAccelXoutH, 16384);
    clock.advanceMs(10);
    ASSERT_EQ(imu.sample(), Status::Ok);

    device::AccelSample before{};
    ASSERT_EQ(imu.read(before), Status::Ok);

    // Advance the clock across the FAILED sample. If the driver stamped a time
    // on a read it never completed, the timestamp check below would catch it.
    clock.advanceMs(10);
    dev.failNextRead = 1;
    EXPECT_EQ(imu.sample(), Status::IoError);
    EXPECT_EQ(imu.health(), device::SensorHealth::Failed);

    device::AccelSample after{};
    ASSERT_EQ(imu.read(after), Status::Ok);
    EXPECT_EQ(after.accel_g.x, before.accel_g.x)
        << "stale is fine and detectable by timestamp; invented data is not";
    EXPECT_EQ(after.time, before.time) << "the timestamp is what makes staleness visible";
}

TEST_F(Mpu6500Test, HealthRecoversAfterATransientBusFault)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    dev.failNextRead = 1;
    ASSERT_EQ(imu.sample(), Status::IoError);
    ASSERT_EQ(imu.health(), device::SensorHealth::Failed);

    EXPECT_EQ(imu.sample(), Status::Ok);
    EXPECT_EQ(imu.health(), device::SensorHealth::Ok)
        << "a single NAK must not latch the sensor off for the rest of the flight";
}

TEST_F(Mpu6500Test, ReportsItsName)
{
    drivers::Mpu6500 imu(dev, clock, scheduler);
    EXPECT_STREQ(imu.name(), "MPU-6500");
}

} // namespace
