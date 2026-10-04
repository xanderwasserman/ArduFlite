/**
 * test_bmm350.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the BMM350 driver (DFRobot SEN0697 10-DOF).
 *
 * This part carries more failure surface than the other two on the module. The
 * BMP581 compensates itself and the BMI323 needs only correct framing, but the
 * BMM350 hands over raw counts plus a block of factory trimming that the driver
 * has to fetch through a command/status handshake, unpack out of shared 16-bit
 * OTP words, and apply. A wrong shift in the unpacking yields a heading that is
 * merely wrong, not obviously broken — which is the worst kind, because the
 * fusion filter will happily drag yaw towards it.
 *
 * So the tests split into three groups: framing, OTP unpacking (bit-exact
 * against hand-computed values), and the compensation chain (each term isolated
 * so a misplaced one cannot hide behind another).
 */
#include <gtest/gtest.h>

#include "hal_host/HostPlatform.h"
#include "src/hal/drivers/mag/Bmm350.h"

using namespace arduflite;
using arduflite::hal::host::FakeRegisterDevice;
using arduflite::hal::host::RecordingScheduler;
using arduflite::hal::host::VirtualClock;

namespace {

using Bmm = drivers::Bmm350;

class Bmm350Test : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Every read on this part is preceded by two filler bytes on I2C. The
        // fake generates them on the bus rather than storing them, so that a
        // read-modify-write still addresses the register it names.
        dev.dummyPrefixBytes = Bmm::kI2cDummyBytes;
        dev.regs[Bmm::kRegChipId] = Bmm::kChipId;

        installOtpEmulation();
    }

    /**
     * @brief Answer the OTP handshake the way the part does.
     *
     * The driver writes an address to OTP_CMD, polls OTP_STATUS for the
     * done bit, then reads MSB and LSB. Emulating that here — rather than
     * pinning status permanently high — is what makes the "driver honours
     * the handshake" tests mean anything.
     */
    void installOtpEmulation()
    {
        dev.onRead = [this](std::uint8_t reg) {
            if (reg == Bmm::kRegOtpStatus)
            {
                dev.regs[Bmm::kRegOtpStatus] = Bmm::kOtpStatusCmdDone;
            }
            else if (reg == Bmm::kRegOtpDataMsb || reg == Bmm::kRegOtpDataLsb)
            {
                const std::uint8_t addr =
                    dev.regs[Bmm::kRegOtpCmd] & Bmm::kOtpWordAddrMask;
                dev.regs[Bmm::kRegOtpDataMsb] =
                    static_cast<std::uint8_t>(otp[addr] >> 8);
                dev.regs[Bmm::kRegOtpDataLsb] =
                    static_cast<std::uint8_t>(otp[addr] & 0xFF);
            }
        };
    }

    /// 24-bit little-endian, XLSB first — this part's data layout.
    void setRaw24(std::uint8_t reg, std::int32_t value)
    {
        const auto raw = static_cast<std::uint32_t>(value);
        dev.regs[reg]     = static_cast<std::uint8_t>(raw & 0xFF);
        dev.regs[reg + 1] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
        dev.regs[reg + 2] = static_cast<std::uint8_t>((raw >> 16) & 0xFF);
    }

    void setField(std::int32_t x, std::int32_t y, std::int32_t z)
    {
        setRaw24(Bmm::kRegMagXXlsb + 0, x);
        setRaw24(Bmm::kRegMagXXlsb + 3, y);
        setRaw24(Bmm::kRegMagXXlsb + 6, z);
    }

    void setRawTemperature(std::int32_t t) { setRaw24(Bmm::kRegMagXXlsb + 9, t); }

    /// Trimming that leaves a sample untouched: the identity against which each
    /// compensation term below is switched on one at a time.
    static Bmm::Compensation identity()
    {
        Bmm::Compensation c{};
        c.t0 = 0.0f;
        return c;
    }

    [[nodiscard]] bool wroteValue(std::uint8_t reg, std::uint8_t value) const
    {
        for (const auto& w : dev.writeLog)
        {
            if (w.reg == reg && w.value == value) { return true; }
        }
        return false;
    }

    std::uint16_t      otp[Bmm::kOtpWordCount]{};
    FakeRegisterDevice dev;
    VirtualClock       clock;
    RecordingScheduler scheduler;
};

// ── Identification ──────────────────────────────────────────────────────────

TEST_F(Bmm350Test, ProbeAcceptsTheCorrectChipId)
{
    Bmm mag(dev, clock, scheduler);
    EXPECT_EQ(mag.probe(), Status::Ok);
    EXPECT_EQ(mag.chipId(), 0x33);
    EXPECT_EQ(mag.health(), device::SensorHealth::Ok);
}

TEST_F(Bmm350Test, ProbeRecordsAWrongIdRatherThanDiscardingIt)
{
    dev.regs[Bmm::kRegChipId] = 0x43;   // a BMI323's low ID byte

    Bmm mag(dev, clock, scheduler);
    EXPECT_EQ(mag.probe(), Status::NotPresent);
    EXPECT_EQ(mag.chipId(), 0x43) << "which part answered is the diagnosis";
}

/// A bus failure and a wrong part are different faults and must stay so: one is
/// a broken module, the other is a mis-declared board descriptor.
TEST_F(Bmm350Test, ProbeReportsNotPresentWhenTheBusFails)
{
    dev.failNextRead = 1;

    Bmm mag(dev, clock, scheduler);
    EXPECT_EQ(mag.probe(), Status::NotPresent);
    EXPECT_EQ(mag.health(), device::SensorHealth::NotPresent);
}

/// The fake fills the prefix with 0xAA. If the driver forgot to skip it, the
/// chip ID would read 0xAA and this test would say so.
TEST_F(Bmm350Test, ReadsSkipTheTwoDummyBytes)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.probe(), Status::Ok);
    EXPECT_NE(mag.chipId(), 0xAA);
}

// ── Bring-up sequence ───────────────────────────────────────────────────────

TEST_F(Bmm350Test, BeginSoftResetsBeforeReadingTrimming)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);

    ASSERT_FALSE(dev.writeLog.empty());
    EXPECT_EQ(dev.writeLog.front().reg, Bmm::kRegCmd);
    EXPECT_EQ(dev.writeLog.front().value, Bmm::kCmdSoftReset);
}

/// 32 words, each fetched by address. Fetching fewer would silently leave the
/// tail of the compensation block at zero.
TEST_F(Bmm350Test, BeginReadsEveryOtpWord)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);

    for (std::size_t i = 0; i < Bmm::kOtpWordCount; ++i)
    {
        const auto command =
            static_cast<std::uint8_t>(Bmm::kOtpCmdDirRead | (i & Bmm::kOtpWordAddrMask));
        EXPECT_TRUE(wroteValue(Bmm::kRegOtpCmd, command))
            << "OTP word " << i << " was never requested";
    }
}

/// The OTP array is only needed at boot. Leaving it powered costs current for
/// the rest of the flight.
TEST_F(Bmm350Test, BeginPowersTheOtpDownWhenDone)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);
    EXPECT_TRUE(wroteValue(Bmm::kRegOtpCmd, Bmm::kOtpCmdPowerOff));
}

/// Bit-reset then flux-guide. Without it the part keeps whatever remanent
/// offset the last strong field left in it — on an airframe, the last time a
/// motor ran near it.
TEST_F(Bmm350Test, BeginPerformsTheMagneticResetBeforeGoingToNormalMode)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);

    int brIndex = -1, fgrIndex = -1, normalIndex = -1;
    for (std::size_t i = 0; i < dev.writeLog.size(); ++i)
    {
        if (dev.writeLog[i].reg != Bmm::kRegPmuCmd) { continue; }
        const int at = static_cast<int>(i);
        if (dev.writeLog[i].value == 0x07)                { brIndex = at; }
        else if (dev.writeLog[i].value == 0x05)           { fgrIndex = at; }
        else if (dev.writeLog[i].value == Bmm::kPmuCmdNormal) { normalIndex = at; }
    }

    ASSERT_GE(brIndex, 0)     << "bit reset never issued";
    ASSERT_GE(fgrIndex, 0)    << "flux guide reset never issued";
    ASSERT_GE(normalIndex, 0) << "part never entered normal mode";
    EXPECT_LT(brIndex, fgrIndex);
    EXPECT_LT(fgrIndex, normalIndex);
}

/// Pinned as a byte, not as a formula: 100 Hz with 4x averaging is 0x24, and
/// the averaging field starts at bit 4. Composing the byte wrongly gives a
/// working magnetometer at the wrong rate, which only shows up as heading lag.
TEST_F(Bmm350Test, BeginConfiguresOneHundredHertzWithFourfoldAveraging)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);

    EXPECT_TRUE(wroteValue(Bmm::kRegPmuCmdAggr, 0x24));
    EXPECT_TRUE(wroteValue(Bmm::kRegPmuCmdAxisEn, Bmm::kEnableXyz));
    EXPECT_EQ(mag.nativeRate_hz(), 100);
}

TEST_F(Bmm350Test, BeginFailsWhenTheOtpHandshakeNeverCompletes)
{
    dev.onRead = [this](std::uint8_t reg) {
        if (reg == Bmm::kRegOtpStatus) { dev.regs[Bmm::kRegOtpStatus] = 0x00; }
    };

    Bmm mag(dev, clock, scheduler);
    EXPECT_EQ(mag.begin(), Status::Timeout);
}

/// The status register reports errors in bits 5..1. Polling through one would
/// spin for the full timeout and then report the wrong fault.
TEST_F(Bmm350Test, BeginFailsImmediatelyOnAnOtpError)
{
    dev.onRead = [this](std::uint8_t reg) {
        if (reg == Bmm::kRegOtpStatus) { dev.regs[Bmm::kRegOtpStatus] = 0x20; }
    };

    Bmm mag(dev, clock, scheduler);
    EXPECT_EQ(mag.begin(), Status::IoError);
}

// ── OTP unpacking ───────────────────────────────────────────────────────────

/**
 * Hand-computed against Bosch's packing. Several coefficients share a word and
 * take either its low or high byte, and the offsets straddle word boundaries in
 * 12-bit fields — so a single misplaced shift moves a value into a neighbour.
 * Values here are chosen to be asymmetric and to exercise sign extension.
 */
TEST_F(Bmm350Test, UnpacksCompensationCoefficientsBitExactly)
{
    otp[0x0D] = 0x8090;   // temp sens (high) | temp offset (low)
    otp[0x0E] = 0xA123;   // offset X (low 12) | offset Y (high nibble)
    otp[0x0F] = 0x0B45;   // offset Y (low) | offset Z (nibble)
    otp[0x10] = 0x7F10;   // offset Z (low) | sens X (high)
    otp[0x11] = 0x4020;   // sens Y (low) | sens Z (high)
    otp[0x15] = 0x1008;   // cross XY (low) | cross YX (high)
    otp[0x18] = 0x0200;   // DUT T0

    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);
    const auto& c = mag.compensation();

    EXPECT_FLOAT_EQ(c.offsetX, 291.0f);      // 0x123, positive
    EXPECT_FLOAT_EQ(c.offsetY, -1467.0f);    // 0xA45 = 2629 -> sign-extended
    EXPECT_FLOAT_EQ(c.offsetZ, -1264.0f);    // 0xB10 = 2832 -> sign-extended
    EXPECT_FLOAT_EQ(c.tempOffset, -22.4f);   // 0x90 = -112, /5
    EXPECT_FLOAT_EQ(c.tempSens, -0.25f);     // 0x80 = -128, /512

    EXPECT_FLOAT_EQ(c.sensX, 127.0f / 256.0f);
    EXPECT_FLOAT_EQ(c.sensY, 32.0f / 256.0f + 0.01f) << "Y carries a fixed correction";
    EXPECT_FLOAT_EQ(c.sensZ, 64.0f / 256.0f);

    EXPECT_FLOAT_EQ(c.crossXY, 8.0f / 800.0f);
    EXPECT_FLOAT_EQ(c.crossYX, 16.0f / 800.0f);

    EXPECT_FLOAT_EQ(c.t0, 24.0f) << "512/512 + 23";
}

/// Z's TCS carries a fixed correction the way Y's sensitivity does. Dropping it
/// is invisible at room temperature and grows with the delta.
TEST_F(Bmm350Test, AppliesTheFixedCorrectionToZTemperatureCoefficient)
{
    otp[0x14] = 0x0000;

    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);
    EXPECT_FLOAT_EQ(mag.compensation().tcsZ, -0.0001f);
}

// ── Framing and scaling ─────────────────────────────────────────────────────

/// X, Y, Z, then temperature — twelve bytes in one burst. A driver that read
/// temperature first would corrupt every axis AND the temperature correction.
TEST_F(Bmm350Test, ReadsTwelveBytesInOneBurstStartingAtTheXAxis)
{
    Bmm mag(dev, clock, scheduler);
    dev.writeLog.clear();
    dev.readCount = 0;

    ASSERT_EQ(mag.sample(), Status::Ok);
    EXPECT_EQ(dev.readCount, 1) << "one burst per tick, not four";
    EXPECT_EQ(dev.lastReadLen, Bmm::kI2cDummyBytes + 12);
}

TEST_F(Bmm350Test, ScalesRawCountsToMicrotesla)
{
    Bmm mag(dev, clock, scheduler);
    mag.setCompensationForTest(identity());

    setField(1000, -2000, 3000);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);

    EXPECT_NEAR(sample.field_ut.x,  1000.0f * Bmm::kLsbToUtXy, 1e-4f);
    EXPECT_NEAR(sample.field_ut.y, -2000.0f * Bmm::kLsbToUtXy, 1e-4f);
    EXPECT_NEAR(sample.field_ut.z,  3000.0f * Bmm::kLsbToUtZ,  1e-4f);
}

/// Z has its own scale factor. Reusing the XY one gives a field that looks fine
/// on the bench and tilts the horizontal projection as soon as the wing banks.
TEST_F(Bmm350Test, ScalesZDifferentlyFromXAndY)
{
    EXPECT_NE(Bmm::kLsbToUtXy, Bmm::kLsbToUtZ);

    Bmm mag(dev, clock, scheduler);
    mag.setCompensationForTest(identity());

    setField(1000, 0, 1000);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);
    EXPECT_GT(sample.field_ut.z, sample.field_ut.x)
        << "Z's divisor is smaller, so the same count is a larger field";
}

/// 24-bit two's complement. Treating it as unsigned turns a small negative
/// field into an enormous positive one.
TEST_F(Bmm350Test, SignExtendsTwentyFourBitCounts)
{
    Bmm mag(dev, clock, scheduler);
    mag.setCompensationForTest(identity());

    setField(-1, 0, 0);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);
    EXPECT_NEAR(sample.field_ut.x, -1.0f * Bmm::kLsbToUtXy, 1e-6f);
}

// ── Compensation chain ──────────────────────────────────────────────────────

TEST_F(Bmm350Test, AppliesOffsetAfterSensitivity)
{
    Bmm mag(dev, clock, scheduler);
    auto comp = identity();
    comp.sensX   = 1.0f;    // doubles
    comp.offsetX = 5.0f;    // then adds
    mag.setCompensationForTest(comp);

    setField(1000, 0, 0);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);

    const float scaled = 1000.0f * Bmm::kLsbToUtXy;
    EXPECT_NEAR(sample.field_ut.x, scaled * 2.0f + 5.0f, 1e-4f)
        << "offset is in compensated units, not raw counts";
}

/// Temperature is compensated first, because every magnetic term is corrected
/// against the delta from T0. Getting the order wrong makes the correction
/// track an uncorrected temperature.
TEST_F(Bmm350Test, CompensatesTemperatureBeforeUsingItAsAReference)
{
    Bmm mag(dev, clock, scheduler);
    auto comp = identity();
    comp.tempSens   = 1.0f;    // doubles
    comp.tempOffset = 10.0f;   // then adds
    mag.setCompensationForTest(comp);

    setField(0, 0, 0);
    setRawTemperature(1000);
    ASSERT_EQ(mag.sample(), Status::Ok);

    const float scaled = 1000.0f * Bmm::kLsbToDegC;
    EXPECT_NEAR(mag.temperature_c(), scaled * 2.0f + 10.0f, 1e-4f);
}

TEST_F(Bmm350Test, AppliesTheTemperatureOffsetCoefficient)
{
    Bmm mag(dev, clock, scheduler);
    auto comp = identity();
    comp.tempOffset = 30.0f;   // raw temp 0 -> 30 degC
    comp.t0         = 20.0f;   // delta of 10
    comp.tcoX       = 0.5f;    // 0.5 uT per degC
    mag.setCompensationForTest(comp);

    setField(1000, 0, 0);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);

    const float scaled = 1000.0f * Bmm::kLsbToUtXy;
    EXPECT_NEAR(sample.field_ut.x, scaled + 5.0f, 1e-4f);
}

/// TCS divides where TCO adds. Swapping them is dimensionally plausible and
/// numerically close near T0, which is exactly where a bench check happens.
TEST_F(Bmm350Test, AppliesTheTemperatureSensitivityCoefficientAsADivisor)
{
    Bmm mag(dev, clock, scheduler);
    auto comp = identity();
    comp.tempOffset = 30.0f;
    comp.t0         = 20.0f;   // delta of 10
    comp.tcsX       = 0.1f;    // divisor becomes 2
    mag.setCompensationForTest(comp);

    setField(1000, 0, 0);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);

    const float scaled = 1000.0f * Bmm::kLsbToUtXy;
    EXPECT_NEAR(sample.field_ut.x, scaled / 2.0f, 1e-4f);
}

/// Neither coefficient may act when the part is at its calibration temperature.
TEST_F(Bmm350Test, LeavesTheSampleAloneAtTheCalibrationTemperature)
{
    Bmm mag(dev, clock, scheduler);
    auto comp = identity();
    comp.tempOffset = 25.0f;
    comp.t0         = 25.0f;   // delta of zero
    comp.tcoX       = 9.0f;    // large enough to be unmissable
    comp.tcsX       = 9.0f;
    mag.setCompensationForTest(comp);

    setField(1000, 0, 0);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);
    EXPECT_NEAR(sample.field_ut.x, 1000.0f * Bmm::kLsbToUtXy, 1e-4f);
}

/// X and Y couple into each other, so the correction is a 2x2 inverse, not two
/// independent subtractions.
TEST_F(Bmm350Test, DecouplesTheHorizontalAxes)
{
    Bmm mag(dev, clock, scheduler);
    auto comp = identity();
    comp.crossXY = 0.5f;
    mag.setCompensationForTest(comp);

    setField(1000, 1000, 0);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);

    const float scaled = 1000.0f * Bmm::kLsbToUtXy;
    EXPECT_NEAR(sample.field_ut.x, scaled - 0.5f * scaled, 1e-4f);
    EXPECT_NEAR(sample.field_ut.y, scaled, 1e-4f) << "crossYX is zero here";
}

TEST_F(Bmm350Test, DecouplesTheVerticalAxisFromBothHorizontalOnes)
{
    Bmm mag(dev, clock, scheduler);
    auto comp = identity();
    comp.crossZX = 0.25f;
    mag.setCompensationForTest(comp);

    setField(1000, 0, 1000);
    setRawTemperature(0);
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);

    const float x = 1000.0f * Bmm::kLsbToUtXy;
    const float z = 1000.0f * Bmm::kLsbToUtZ;
    EXPECT_NEAR(sample.field_ut.z, z - 0.25f * x, 1e-4f);
}

// ── Health and the sample/read split ────────────────────────────────────────

/// The whole point of ADR-032. A failed burst must not leave the fusion filter
/// pulling towards a heading captured before the bus died.
TEST_F(Bmm350Test, MarksItselfFailedWhenTheBurstFails)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);
    ASSERT_EQ(mag.sample(), Status::Ok);
    ASSERT_EQ(mag.health(), device::SensorHealth::Ok);

    dev.failNextRead = 1;
    EXPECT_EQ(mag.sample(), Status::IoError);
    EXPECT_EQ(mag.health(), device::SensorHealth::Failed);
}

TEST_F(Bmm350Test, RecoversHealthOnTheNextGoodBurst)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.begin(), Status::Ok);

    dev.failNextRead = 1;
    ASSERT_EQ(mag.sample(), Status::IoError);

    EXPECT_EQ(mag.sample(), Status::Ok);
    EXPECT_EQ(mag.health(), device::SensorHealth::Ok);
}

/// read() is called from the estimation tick and must never reach the bus —
/// that is what keeps the twelve-step contract free of blocking I/O.
TEST_F(Bmm350Test, ReadNeverTouchesTheBus)
{
    Bmm mag(dev, clock, scheduler);
    ASSERT_EQ(mag.sample(), Status::Ok);

    const int before = dev.transactions;
    device::MagSample sample{};
    for (int i = 0; i < 10; ++i) { EXPECT_EQ(mag.read(sample), Status::Ok); }
    EXPECT_EQ(dev.transactions, before);
}

TEST_F(Bmm350Test, StampsTheSampleWithTheTimeItWasTaken)
{
    Bmm mag(dev, clock, scheduler);

    clock.advance(std::chrono::milliseconds(250));
    ASSERT_EQ(mag.sample(), Status::Ok);

    device::MagSample sample{};
    ASSERT_EQ(mag.read(sample), Status::Ok);
    EXPECT_EQ(sample.time, clock.now());
    EXPECT_NE(sample.time.time_since_epoch().count(), 0)
        << "a clock that never advanced would make this assertion vacuous";
}

} // namespace
