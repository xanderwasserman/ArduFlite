/**
 * test_bmi323.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the BMI323 driver (DFRobot SEN0697 10-DOF).
 *
 * Two of this chip's conventions differ from the MPU-6500 and both fail
 * SILENTLY — wrong numbers, no error:
 *
 *   - every I2C read is preceded by TWO dummy bytes;
 *   - registers are 16-bit LITTLE-endian, where the MPU-6500 was big-endian.
 *
 * Values here come from Bosch's own driver as shipped by DFRobot
 * (bmi3_defs.h, bmi3.c), not from recollection.
 */
#include <gtest/gtest.h>

#include "hal_host/HostPlatform.h"
#include "src/hal/drivers/imu/Bmi323.h"

using namespace arduflite;
using arduflite::hal::host::FakeRegisterDevice;
using arduflite::hal::host::RecordingScheduler;
using arduflite::hal::host::VirtualClock;

namespace {

class Bmi323Test : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // The device prepends two filler bytes to every I2C read, and its
        // registers address 16-bit WORDS — 0x20 and 0x21 are adjacent
        // registers, not adjacent bytes.
        dev.dummyPrefixBytes = drivers::Bmi323::kI2cDummyBytes;
        dev.wordAddressed    = true;
        setWord(drivers::Bmi323::kRegChipId, drivers::Bmi323::kChipId);
    }

    /// 16-bit little-endian register, at its word slot.
    void setWord(std::uint8_t reg, std::uint16_t value)
    {
        dev.regs[reg * 2]     = static_cast<std::uint8_t>(value & 0xFF);
        dev.regs[reg * 2 + 1] = static_cast<std::uint8_t>(value >> 8);
    }

    /**
     * The register's resulting VALUE, not the write log.
     *
     * The log is byte-indexed, so scanning it for `reg` and `reg + 1` picks up
     * the neighbouring REGISTER's low byte as this one's high byte — exactly
     * the confusion word addressing exists to avoid. ACC_CONF (0x20) and
     * GYR_CONF (0x21) sit next to each other, so this bites immediately.
     */
    int registerValue(std::uint8_t reg) const
    {
        return dev.regs[reg * 2] | (dev.regs[reg * 2 + 1] << 8);
    }

    FakeRegisterDevice dev;
    VirtualClock       clock;
    RecordingScheduler scheduler;
};

// ── Identification ──────────────────────────────────────────────────────────

TEST_F(Bmi323Test, ProbeAcceptsTheGenuineChipId)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    EXPECT_EQ(imu.probe(), Status::Ok);
    EXPECT_EQ(imu.chipId() & 0x00FF, 0x43);
}

/// The high byte is a silicon revision that varies between parts. Comparing the
/// whole word would reject valid chips.
TEST_F(Bmi323Test, ProbeIgnoresTheRevisionInTheHighByte)
{
    setWord(drivers::Bmi323::kRegChipId, 0x1143);

    drivers::Bmi323 imu(dev, clock, scheduler);
    EXPECT_EQ(imu.probe(), Status::Ok);
}

TEST_F(Bmi323Test, ProbeRecordsAWrongIdRatherThanDiscardingIt)
{
    setWord(drivers::Bmi323::kRegChipId, 0x0024);   // BMI270's ID

    drivers::Bmi323 imu(dev, clock, scheduler);
    EXPECT_EQ(imu.probe(), Status::NotPresent);
    EXPECT_EQ(imu.chipId() & 0x00FF, 0x24) << "which part answered is the diagnosis";
}

// ── The two silent-corruption cases ─────────────────────────────────────────

/**
 * If the dummy bytes are not discarded, every value shifts by two and the
 * driver returns numbers that look like data. This is the single most likely
 * way to get this chip subtly wrong.
 */
TEST_F(Bmi323Test, DiscardsTheTwoDummyBytesOnEveryRead)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    // Accel X = +8192 counts = half of a 4 g range = 1.0 g. The fake emits two
    // 0xAA filler bytes ahead of it; taking those as data would give ~4 g.
    setWord(drivers::Bmi323::kRegAccDataX, 0x2000);

    ASSERT_EQ(imu.sample(), Status::Ok);

    device::AccelSample accel{};
    ASSERT_EQ(imu.read(accel), Status::Ok);
    EXPECT_NEAR(accel.accel_g.x, 1.0f, 1e-4f)
        << "reading the dummy bytes as data would give ~4 g here";
}

/// Little-endian, unlike the MPU-6500. A byte-swap gives a wrong-but-plausible
/// number, never an error.
TEST_F(Bmi323Test, DecodesWordsLittleEndian)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    // 256 counts. A byte-swap would read 0x0001 = 1 count.
    setWord(drivers::Bmi323::kRegAccDataX, 0x0100);

    ASSERT_EQ(imu.sample(), Status::Ok);
    device::AccelSample accel{};
    ASSERT_EQ(imu.read(accel), Status::Ok);
    EXPECT_NEAR(accel.accel_g.x, 256.0f * 4.0f / 32768.0f, 1e-6f);
}

TEST_F(Bmi323Test, NegativeValuesAreTwosComplement)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    setWord(drivers::Bmi323::kRegAccDataX, 0xFFFF);   // -1

    ASSERT_EQ(imu.sample(), Status::Ok);
    device::AccelSample accel{};
    ASSERT_EQ(imu.read(accel), Status::Ok);
    EXPECT_LT(accel.accel_g.x, 0.0f);
    EXPECT_NEAR(accel.accel_g.x, -4.0f / 32768.0f, 1e-9f);
}

// ── Configuration ───────────────────────────────────────────────────────────

TEST_F(Bmi323Test, BeginSoftResetsFirstAndWaits)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    ASSERT_GE(dev.writeLog.size(), 2u);
    EXPECT_EQ(dev.writeLog[0].reg, drivers::Bmi323::kRegCmd);
    // CMD is write-only, so this one really does check the log — reading it
    // back would say nothing about what was commanded.
    ASSERT_GE(dev.writeLog.size(), 2u);
    EXPECT_EQ(dev.writeLog[0].value | (dev.writeLog[1].value << 8), 0xDEAF)
        << "Bosch's soft-reset command";

    ASSERT_FALSE(scheduler.sleepLog.empty())
        << "the part needs ~1500 us after reset before it will accept configuration";
}

TEST_F(Bmi323Test, BeginSelectsHighPerformanceModeOnBothSensors)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    for (std::uint8_t reg : { drivers::Bmi323::kRegAccConf, drivers::Bmi323::kRegGyrConf })
    {
        const int conf = registerValue(reg);
        ASSERT_GE(conf, 0);
        EXPECT_EQ((conf & drivers::Bmi323::kModeMask) >> drivers::Bmi323::kModePos, 0x07)
            << "a duty-cycled sensor returns repeats to a 500 Hz loop";
        EXPECT_EQ(conf & drivers::Bmi323::kOdrMask, drivers::Bmi323::kOdr400Hz);
    }
}

TEST_F(Bmi323Test, RangesLandInTheRightField)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    EXPECT_EQ((registerValue(drivers::Bmi323::kRegAccConf) & drivers::Bmi323::kRangeMask)
                  >> drivers::Bmi323::kRangePos, 1) << "4 g";
    EXPECT_EQ((registerValue(drivers::Bmi323::kRegGyrConf) & drivers::Bmi323::kRangeMask)
                  >> drivers::Bmi323::kRangePos, 2) << "500 dps";
}

TEST_F(Bmi323Test, SettingARangeDoesNotDisturbTheModeOrOdr)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);
    ASSERT_EQ(imu.setRange_g(16), Status::Ok);

    const int conf = registerValue(drivers::Bmi323::kRegAccConf);
    EXPECT_EQ((conf & drivers::Bmi323::kRangeMask) >> drivers::Bmi323::kRangePos, 3);
    EXPECT_EQ((conf & drivers::Bmi323::kModeMask) >> drivers::Bmi323::kModePos, 0x07)
        << "read-modify-write must preserve the mode";
    EXPECT_EQ(conf & drivers::Bmi323::kOdrMask, drivers::Bmi323::kOdr400Hz);
}

TEST_F(Bmi323Test, RejectsRangesTheHardwareCannotEncode)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    EXPECT_EQ(imu.setRange_g(3), Status::InvalidArg);
    EXPECT_EQ(imu.setRange_dps(400), Status::InvalidArg);
    EXPECT_EQ(imu.range_g(), 4);
    EXPECT_EQ(imu.range_dps(), 500);
}

/// 125 dps exists on this part and not on the MPU-6500.
TEST_F(Bmi323Test, Accepts125DpsWhichTheMpu6500CannotDo)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    EXPECT_EQ(imu.setRange_dps(125), Status::Ok);
    EXPECT_EQ(imu.range_dps(), 125);
}

// ── Sampling contract ───────────────────────────────────────────────────────

TEST_F(Bmi323Test, SampleIsOneBurstAndReadIsFree)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    dev.transactions = 0;
    ASSERT_EQ(imu.sample(), Status::Ok);
    EXPECT_EQ(dev.transactions, 1);
    EXPECT_EQ(dev.lastReadLen, 14u) << "2 dummy + accel(6) + gyro(6)";

    device::AccelSample a{};
    device::GyroSample  g{};
    ASSERT_EQ(imu.read(a), Status::Ok);
    ASSERT_EQ(imu.read(g), Status::Ok);
    EXPECT_EQ(dev.transactions, 1) << "read() must never touch the bus";
}

TEST_F(Bmi323Test, SamplingNeverSleeps)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);

    const std::size_t before = scheduler.sleepLog.size();
    ASSERT_EQ(imu.sample(), Status::Ok);
    EXPECT_EQ(scheduler.sleepLog.size(), before)
        << "sample() runs inside the 500 Hz task";
}

TEST_F(Bmi323Test, BothSamplesShareOneTimestamp)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);
    clock.advanceMs(4321);
    ASSERT_EQ(imu.sample(), Status::Ok);

    device::AccelSample a{};
    device::GyroSample  g{};
    ASSERT_EQ(imu.read(a), Status::Ok);
    ASSERT_EQ(imu.read(g), Status::Ok);

    EXPECT_EQ(a.time, g.time);
    EXPECT_NE(a.time.time_since_epoch().count(), 0);
}

TEST_F(Bmi323Test, FailedSampleIsReportedAndKeepsTheLastGoodValue)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    ASSERT_EQ(imu.begin(), Status::Ok);
    clock.advanceMs(10);
    ASSERT_EQ(imu.sample(), Status::Ok);

    device::AccelSample before{};
    ASSERT_EQ(imu.read(before), Status::Ok);

    clock.advanceMs(10);
    dev.failNextRead = 1;
    EXPECT_EQ(imu.sample(), Status::IoError);
    EXPECT_EQ(imu.health(), device::SensorHealth::Failed);

    device::AccelSample after{};
    ASSERT_EQ(imu.read(after), Status::Ok);
    EXPECT_EQ(after.time, before.time) << "an unchanged timestamp is how staleness shows";
}

TEST_F(Bmi323Test, ReportsItsIdentity)
{
    drivers::Bmi323 imu(dev, clock, scheduler);
    EXPECT_STREQ(imu.name(), "BMI323");
    EXPECT_EQ(imu.nativeRate_hz(), 400);
}

} // namespace
