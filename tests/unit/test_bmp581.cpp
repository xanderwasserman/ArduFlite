/**
 * test_bmp581.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the BMP581 driver (DFRobot SEN0697 10-DOF).
 *
 * The BMP280 needed its compensation polynomial pinned against Bosch's worked
 * example, because a wrong coefficient produced a plausible pressure. The
 * BMP581 compensates internally, so what is left to get wrong is the FRAMING:
 * 24-bit little-endian, temperature before pressure, and two different scale
 * factors. Each of those fails silently.
 */
#include <gtest/gtest.h>

#include "hal_host/HostPlatform.h"
#include "src/hal/drivers/baro/Bmp581.h"

using namespace arduflite;
using arduflite::hal::host::FakeRegisterDevice;
using arduflite::hal::host::RecordingScheduler;
using arduflite::hal::host::VirtualClock;

namespace {

class Bmp581Test : public ::testing::Test
{
protected:
    void SetUp() override { dev.regs[drivers::Bmp581::kRegChipId] = drivers::Bmp581::kChipId; }

    /// 24-bit little-endian, XLSB first — the part's data layout.
    void setRaw24(std::uint8_t reg, std::uint32_t value)
    {
        dev.regs[reg]     = static_cast<std::uint8_t>(value & 0xFF);
        dev.regs[reg + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
        dev.regs[reg + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
    }

    void setTemperature(std::uint32_t raw) { setRaw24(drivers::Bmp581::kRegTempData, raw); }
    void setPressure(std::uint32_t raw)    { setRaw24(drivers::Bmp581::kRegTempData + 3, raw); }

    FakeRegisterDevice dev;
    VirtualClock       clock;
    RecordingScheduler scheduler;
};

// ── Identification ──────────────────────────────────────────────────────────

TEST_F(Bmp581Test, ProbeAcceptsTheCorrectChipId)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.probe(), Status::Ok);
    EXPECT_EQ(baro.chipId(), 0x50);
}

TEST_F(Bmp581Test, ProbeRecordsAWrongIdRatherThanDiscardingIt)
{
    dev.regs[drivers::Bmp581::kRegChipId] = 0x58;   // a BMP280's ID

    drivers::Bmp581 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.probe(), Status::NotPresent);
    EXPECT_EQ(baro.chipId(), 0x58) << "which part answered is the diagnosis";
}

// ── Scaling ─────────────────────────────────────────────────────────────────

/// Pressure is /64, temperature is /65536. Swapping the two divisors gives
/// numbers that are wrong by a factor of 1024 — which looks like a broken
/// sensor, not a broken driver, and is the cheaper mistake to catch here.
TEST_F(Bmp581Test, PressureUsesTheQ18point6Scale)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    // 101325 Pa in Q18.6 = 101325 * 64 = 6484800.
    setPressure(6484800u);
    setTemperature(0);

    ASSERT_EQ(baro.sample(), Status::Ok);
    device::BaroSample s{};
    ASSERT_EQ(baro.read(s), Status::Ok);
    EXPECT_NEAR(s.pressure_pa, 101325.0f, 0.5f) << "sea-level standard pressure";
}

TEST_F(Bmp581Test, TemperatureUsesTheQ8point16Scale)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    setTemperature(25u * 65536u);   // 25.0 degC
    setPressure(6484800u);

    ASSERT_EQ(baro.sample(), Status::Ok);
    device::BaroSample s{};
    ASSERT_EQ(baro.read(s), Status::Ok);
    EXPECT_NEAR(s.temp_c, 25.0f, 0.01f);
}

/// Temperature is SIGNED 24-bit. Read unsigned, -10 degC becomes about +256 degC.
TEST_F(Bmp581Test, NegativeTemperatureSignExtendsFrom24Bits)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    // -10 degC = -655360, as a 24-bit two's-complement value.
    setTemperature(static_cast<std::uint32_t>(-655360) & 0x00FFFFFFu);
    setPressure(6484800u);

    ASSERT_EQ(baro.sample(), Status::Ok);
    device::BaroSample s{};
    ASSERT_EQ(baro.read(s), Status::Ok);
    EXPECT_NEAR(s.temp_c, -10.0f, 0.01f)
        << "reading it unsigned would give roughly +256 degC";
}

/// The BMP280 had pressure first; this part has temperature first. A copied
/// offset swaps them, and both channels then look like broken sensors.
TEST_F(Bmp581Test, TemperatureComesBeforePressureInTheBurst)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    setTemperature(20u * 65536u);   // 20 degC
    setPressure(6400000u);          // 100000 Pa

    ASSERT_EQ(baro.sample(), Status::Ok);
    device::BaroSample s{};
    ASSERT_EQ(baro.read(s), Status::Ok);

    EXPECT_NEAR(s.temp_c, 20.0f, 0.01f);
    EXPECT_NEAR(s.pressure_pa, 100000.0f, 1.0f);
}

// ── Configuration ───────────────────────────────────────────────────────────

TEST_F(Bmp581Test, BeginSoftResetsFirst)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    ASSERT_FALSE(dev.writeLog.empty());
    EXPECT_EQ(dev.writeLog[0].reg, drivers::Bmp581::kRegCmd);
    EXPECT_EQ(dev.writeLog[0].value, 0xB6);
    EXPECT_FALSE(scheduler.sleepLog.empty()) << "the part needs time to restart";
}

TEST_F(Bmp581Test, BeginEnablesPressureAndSelectsNormalMode)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    const std::uint8_t osr = dev.regs[drivers::Bmp581::kRegOsrConfig];
    EXPECT_EQ((osr >> drivers::Bmp581::kPressEnPos) & 1, 1)
        << "without press_en the pressure registers never update";
    EXPECT_EQ((osr >> drivers::Bmp581::kOsrPressPos) & 0x07, 4) << "pressure x16";
    EXPECT_EQ((osr >> drivers::Bmp581::kOsrTempPos) & 0x07, 0)  << "temperature x1";

    const std::uint8_t odr = dev.regs[drivers::Bmp581::kRegOdrConfig];
    EXPECT_EQ((odr >> drivers::Bmp581::kPwrModePos) & 0x03, 1) << "normal mode";
    EXPECT_EQ((odr >> drivers::Bmp581::kOdrPos) & 0x1F, 0x0F)  << "50 Hz";
    EXPECT_EQ((odr >> drivers::Bmp581::kDeepDisPos) & 1, 1)
        << "deep sleep would duty-cycle the part and feed the filter repeats";
}

TEST_F(Bmp581Test, BeginPropagatesBusFailure)
{
    dev.failNextWrite = 1;

    drivers::Bmp581 baro(dev, clock, scheduler);
    EXPECT_EQ(baro.begin(), Status::IoError);
}

// ── Sampling contract ───────────────────────────────────────────────────────

TEST_F(Bmp581Test, SampleIsOneBurstAndReadIsFree)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    dev.transactions = 0;
    ASSERT_EQ(baro.sample(), Status::Ok);
    EXPECT_EQ(dev.transactions, 1);
    EXPECT_EQ(dev.lastReadLen, 6u) << "temperature(3) + pressure(3)";

    device::BaroSample s{};
    ASSERT_EQ(baro.read(s), Status::Ok);
    ASSERT_EQ(baro.read(s), Status::Ok);
    EXPECT_EQ(dev.transactions, 1);
}

TEST_F(Bmp581Test, SamplingNeverSleeps)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);

    const std::size_t before = scheduler.sleepLog.size();
    ASSERT_EQ(baro.sample(), Status::Ok);
    EXPECT_EQ(scheduler.sleepLog.size(), before);
}

TEST_F(Bmp581Test, FailedSampleKeepsTheLastGoodValue)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    ASSERT_EQ(baro.begin(), Status::Ok);
    setPressure(6484800u);
    clock.advanceMs(10);
    ASSERT_EQ(baro.sample(), Status::Ok);

    device::BaroSample before{};
    ASSERT_EQ(baro.read(before), Status::Ok);

    clock.advanceMs(10);
    dev.failNextRead = 1;
    EXPECT_EQ(baro.sample(), Status::IoError);
    EXPECT_EQ(baro.health(), device::SensorHealth::Failed);

    device::BaroSample after{};
    ASSERT_EQ(baro.read(after), Status::Ok);
    EXPECT_EQ(after.pressure_pa, before.pressure_pa);
    EXPECT_EQ(after.time, before.time);
}

TEST_F(Bmp581Test, ReportsARateTheDecimatorCanUse)
{
    drivers::Bmp581 baro(dev, clock, scheduler);
    EXPECT_STREQ(baro.name(), "BMP581");
    EXPECT_EQ(baro.nativeRate_hz(), 50)
        << "InertialSubsystem decimates its reads against this (ADR-046)";
}

} // namespace
