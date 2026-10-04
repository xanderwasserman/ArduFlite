/**
 * test_bmp280.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the BMP280 driver.
 *
 * The compensation polynomial is the reason these exist. A transposed
 * coefficient or a wrong shift does not crash and does not look obviously wrong
 * on a bench — it produces a pressure that is off by a few hundred Pa, i.e. tens
 * of metres of altitude, discovered in flight. Bosch publishes a worked example
 * precisely so this can be checked; it is the anchor for this file.
 */
#include <gtest/gtest.h>

#include "hal_host/HostPlatform.h"
#include "src/hal/drivers/baro/Bmp280.h"

using namespace arduflite;
using arduflite::hal::host::FakeRegisterDevice;
using arduflite::hal::host::RecordingScheduler;
using arduflite::hal::host::VirtualClock;

namespace {

/**
 * The reference trimming values printed in BST-BMP280-DS001 §3.11.3, alongside
 * adc_T = 519888 / adc_P = 415148 -> 25.08 degC and 100653 Pa.
 */
constexpr drivers::Bmp280::Calibration kDatasheetCalibration{
    .t1 = 27504, .t2 = 26435, .t3 = -1000,
    .p1 = 36477, .p2 = -10685, .p3 = 3024, .p4 = 2855,
    .p5 = 140, .p6 = -7, .p7 = 15500, .p8 = -14600, .p9 = 6000
};

constexpr std::int32_t kDatasheetRawTemperature = 519888;
constexpr std::int32_t kDatasheetRawPressure    = 415148;

class Bmp280Test : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dev.regs[drivers::Bmp280::kRegId] = drivers::Bmp280::kIdExpected;
        writeCalibration(kDatasheetCalibration);
    }

    /// Lay the trimming block out at 0x88 the way the part does: little-endian.
    void writeCalibration(const drivers::Bmp280::Calibration& c)
    {
        const std::uint16_t words[12] = {
            c.t1, static_cast<std::uint16_t>(c.t2), static_cast<std::uint16_t>(c.t3),
            c.p1, static_cast<std::uint16_t>(c.p2), static_cast<std::uint16_t>(c.p3),
            static_cast<std::uint16_t>(c.p4), static_cast<std::uint16_t>(c.p5),
            static_cast<std::uint16_t>(c.p6), static_cast<std::uint16_t>(c.p7),
            static_cast<std::uint16_t>(c.p8), static_cast<std::uint16_t>(c.p9)
        };
        for (int i = 0; i < 12; ++i)
        {
            dev.regs[drivers::Bmp280::kRegCalibration + i * 2]     = words[i] & 0xFF;
            dev.regs[drivers::Bmp280::kRegCalibration + i * 2 + 1] = (words[i] >> 8) & 0xFF;
        }
    }

    /// Store a 20-bit reading left-aligned across three bytes, as the part does.
    void writeRaw20(std::uint8_t reg, std::int32_t value)
    {
        dev.regs[reg]     = static_cast<std::uint8_t>((value >> 12) & 0xFF);
        dev.regs[reg + 1] = static_cast<std::uint8_t>((value >> 4) & 0xFF);
        dev.regs[reg + 2] = static_cast<std::uint8_t>((value << 4) & 0xF0);
    }

    FakeRegisterDevice dev;
    VirtualClock       clock;
    RecordingScheduler scheduler;
};

// ── The anchor ──────────────────────────────────────────────────────────────

TEST(Bmp280Compensation, MatchesTheBoschDatasheetWorkedExample)
{
    std::int32_t fine = 0;
    const std::int32_t t = drivers::Bmp280::compensateTemperature(
        kDatasheetCalibration, kDatasheetRawTemperature, fine);

    EXPECT_EQ(t, 2508) << "datasheet states 25.08 degC";
    EXPECT_EQ(fine, 128422) << "t_fine feeds pressure; a wrong value corrupts both";

    const std::uint32_t p = drivers::Bmp280::compensatePressure(
        kDatasheetCalibration, kDatasheetRawPressure, fine);

    // Q24.8: the datasheet quotes 25767233 / 256 = 100653.25 Pa.
    EXPECT_NEAR(static_cast<double>(p) / 256.0, 100653.0, 1.0);
}

TEST(Bmp280Compensation, UncalibratedPartYieldsZeroRatherThanDividingByZero)
{
    drivers::Bmp280::Calibration bad{};   // p1 == 0 makes var1 collapse to 0
    EXPECT_EQ(drivers::Bmp280::compensatePressure(bad, kDatasheetRawPressure, 128422), 0u);
}

/**
 * The raw count runs INVERSE to pressure: the polynomial starts from
 * (1048576 - adc_P), so a larger reading is a lower pressure. A sign slip here
 * would invert the altitude channel while still passing the single-point
 * datasheet check above, so the direction is pinned explicitly.
 */
TEST(Bmp280Compensation, RawCountRunsInverseToPressure)
{
    std::int32_t fine = 0;
    drivers::Bmp280::compensateTemperature(kDatasheetCalibration,
                                           kDatasheetRawTemperature, fine);

    const auto at = [&](std::int32_t raw) {
        return drivers::Bmp280::compensatePressure(kDatasheetCalibration, raw, fine) / 256.0;
    };

    EXPECT_GT(at(kDatasheetRawPressure - 20000), at(kDatasheetRawPressure));
    EXPECT_LT(at(kDatasheetRawPressure + 20000), at(kDatasheetRawPressure));

    // ~3.45 kPa per 20000 counts, i.e. roughly 290 m of altitude. A gain error
    // of a factor of two or ten shows up here even though the direction is right.
    EXPECT_NEAR(at(kDatasheetRawPressure - 20000) - at(kDatasheetRawPressure), 3454.0, 20.0);
}

// ── Identification and configuration ────────────────────────────────────────

TEST_F(Bmp280Test, ProbeAcceptsTheCorrectChipId)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.probe(), Status::Ok);
    EXPECT_EQ(baro.chipId(), 0x58);
}

TEST_F(Bmp280Test, ProbeRecordsTheIdOfTheWrongPart)
{
    dev.regs[drivers::Bmp280::kRegId] = 0x60;   // BME280

    drivers::Bmp280 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.probe(), Status::NotPresent);
    EXPECT_EQ(baro.chipId(), 0x60)
        << "a BME280 answers on the same address; the log must be able to say so";
}

TEST_F(Bmp280Test, BeginResetsThenReadsTrimmingThenConfigures)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    ASSERT_FALSE(dev.writeLog.empty());
    EXPECT_EQ(dev.writeLog[0].reg, drivers::Bmp280::kRegReset);
    EXPECT_EQ(dev.writeLog[0].value, 0xB6)
        << "trimming is reloaded from NVM by the reset, so it must come first";

    EXPECT_EQ(baro.calibration().t1, kDatasheetCalibration.t1);
    EXPECT_EQ(baro.calibration().p9, kDatasheetCalibration.p9)
        << "the last word of the block - catches an off-by-one read length";
    EXPECT_EQ(baro.calibration().t3, -1000) << "signed words must stay signed";
    EXPECT_EQ(baro.calibration().p6, -7);
}

/**
 * Pins the register bytes against Adafruit_BMP280's defaults, which are what has
 * actually been flying. An earlier revision of this driver quietly used
 * temperature x2 and switched the hardware IIR filter ON; that would have put a
 * second filter in series with ArduFliteIMU's own altitude EMA and changed the
 * vario's dynamics, in a phase whose entire premise is "same numbers, different
 * code path". Hence exact byte assertions rather than "looks reasonable".
 */
TEST_F(Bmp280Test, BeginReproducesTheLegacyAdafruitConfiguration)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    // Adafruit_BMP280::setSampling() defaults: MODE_NORMAL, temp x16, press x16,
    // FILTER_OFF, t_sb = 0.
    EXPECT_EQ(dev.regs[drivers::Bmp280::kRegCtrlMeas], 0xB7)
        << "temp x16 | press x16 | normal mode";
    EXPECT_EQ(dev.regs[drivers::Bmp280::kRegConfig], 0x00)
        << "IIR filter OFF - AltitudeFilter already smooths altitude in software";

    const std::uint8_t ctrl = dev.regs[drivers::Bmp280::kRegCtrlMeas];
    EXPECT_EQ(ctrl & 0x03, 0x03) << "normal mode - forced mode samples once and stops";
}

TEST_F(Bmp280Test, BeginWritesConfigBeforeCtrlMeas)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    int configIndex = -1, ctrlIndex = -1;
    for (std::size_t i = 0; i < dev.writeLog.size(); ++i)
    {
        if (dev.writeLog[i].reg == drivers::Bmp280::kRegConfig)   { configIndex = static_cast<int>(i); }
        if (dev.writeLog[i].reg == drivers::Bmp280::kRegCtrlMeas) { ctrlIndex   = static_cast<int>(i); }
    }
    ASSERT_GE(configIndex, 0);
    ASSERT_GE(ctrlIndex, 0);
    EXPECT_LT(configIndex, ctrlIndex)
        << "CONFIG writes can be dropped once the part is in normal mode";
}

/// A part that ACKs but returns an empty trimming block is worse than one that
/// does not answer: compensation happily produces a plausible constant.
TEST_F(Bmp280Test, BeginRejectsAnEmptyCalibrationBlock)
{
    for (int i = 0; i < 24; ++i) { dev.regs[drivers::Bmp280::kRegCalibration + i] = 0; }

    drivers::Bmp280 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.begin(), Status::IoError);
    EXPECT_EQ(baro.health(), device::SensorHealth::Failed);
}

/**
 * The trimming coefficients are copied from NVM after a reset, and the block
 * reads back garbage until STATUS bit 0 clears. Reading early does not fail —
 * it yields plausible numbers that are simply wrong, and every pressure derived
 * from them is wrong for the life of the flight. Nothing on a bench would flag
 * it.
 */
TEST_F(Bmp280Test, BeginWaitsForTheNvmCopyBeforeReadingTrimming)
{
    // Report "still copying" for the first three status reads, then done.
    dev.regs[drivers::Bmp280::kRegStatus] = drivers::Bmp280::kStatusImUpdate;
    int statusReads = 0;
    dev.onRead = [&](std::uint8_t reg) {
        if (reg == drivers::Bmp280::kRegStatus && ++statusReads >= 4)
        {
            dev.regs[drivers::Bmp280::kRegStatus] = 0;
        }
        // If the driver reads trimming while im_update is still asserted, poison
        // it so the assertion below cannot pass by accident.
        if (reg == drivers::Bmp280::kRegCalibration &&
            (dev.regs[drivers::Bmp280::kRegStatus] & drivers::Bmp280::kStatusImUpdate))
        {
            for (int i = 0; i < 24; ++i) { dev.regs[drivers::Bmp280::kRegCalibration + i] = 0xFF; }
        }
    };

    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    EXPECT_GE(statusReads, 4) << "must poll, not assume";
    EXPECT_EQ(baro.calibration().t1, kDatasheetCalibration.t1)
        << "trimming was read before the copy finished";
}

TEST_F(Bmp280Test, BeginGivesUpIfTheNvmCopyNeverCompletes)
{
    dev.regs[drivers::Bmp280::kRegStatus] = drivers::Bmp280::kStatusImUpdate;   // stuck

    drivers::Bmp280 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.begin(), Status::Timeout)
        << "better to report no barometer than one that reads confidently and wrongly";
    EXPECT_EQ(baro.health(), device::SensorHealth::Failed);
}

TEST_F(Bmp280Test, SamplingNeverSleeps)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    const std::size_t afterBegin = scheduler.sleepLog.size();
    ASSERT_EQ(baro.sample(), Status::Ok);
    EXPECT_EQ(scheduler.sleepLog.size(), afterBegin)
        << "sample() is called from the IMU task's critical section";
}

TEST_F(Bmp280Test, BeginPropagatesBusFailure)
{
    dev.failNextWrite = 1;

    drivers::Bmp280 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.begin(), Status::IoError);
}

// ── Sampling ────────────────────────────────────────────────────────────────

TEST_F(Bmp280Test, SampleIsOneTransactionAndReadIsNone)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    dev.transactions = 0;
    ASSERT_EQ(baro.sample(), Status::Ok);
    EXPECT_EQ(dev.transactions, 1);
    EXPECT_EQ(dev.lastReadLen, 6u) << "pressure(3) + temperature(3) in one burst";

    device::BaroSample s{};
    ASSERT_EQ(baro.read(s), Status::Ok);
    ASSERT_EQ(baro.read(s), Status::Ok);
    EXPECT_EQ(dev.transactions, 1) << "read() must never touch the bus";
}

/// The datasheet example again, but through the whole driver: register layout,
/// 20-bit unpacking, compensation and unit conversion together.
TEST_F(Bmp280Test, EndToEndSampleReproducesTheDatasheetValues)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    writeRaw20(drivers::Bmp280::kRegPressMsb,     kDatasheetRawPressure);
    writeRaw20(drivers::Bmp280::kRegPressMsb + 3, kDatasheetRawTemperature);

    ASSERT_EQ(baro.sample(), Status::Ok);

    device::BaroSample s{};
    ASSERT_EQ(baro.read(s), Status::Ok);
    EXPECT_NEAR(s.temp_c, 25.08f, 0.01f);
    EXPECT_NEAR(s.pressure_pa, 100653.0f, 1.0f)
        << "reported in Pa; InertialSubsystem divides by 100 for hPa";
}

TEST_F(Bmp280Test, FailedSampleIsReportedAndLeavesTheLastGoodValue)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);
    writeRaw20(drivers::Bmp280::kRegPressMsb,     kDatasheetRawPressure);
    writeRaw20(drivers::Bmp280::kRegPressMsb + 3, kDatasheetRawTemperature);
    ASSERT_EQ(baro.sample(), Status::Ok);

    device::BaroSample before{};
    ASSERT_EQ(baro.read(before), Status::Ok);

    dev.failNextRead = 1;
    EXPECT_EQ(baro.sample(), Status::IoError);
    EXPECT_EQ(baro.health(), device::SensorHealth::Failed);

    device::BaroSample after{};
    ASSERT_EQ(baro.read(after), Status::Ok);
    EXPECT_EQ(after.pressure_pa, before.pressure_pa);
    EXPECT_EQ(after.time, before.time)
        << "an unchanged timestamp is how altitude hold detects a stale baro";
}

TEST_F(Bmp280Test, ReportsItsNameAndRate)
{
    drivers::Bmp280 baro(dev, clock, scheduler);
    EXPECT_STREQ(baro.name(), "BMP280");
    EXPECT_EQ(baro.nativeRate_hz(), 15)
        << "x16/x16 with the filter off is ~66 ms per conversion";
}

} // namespace
