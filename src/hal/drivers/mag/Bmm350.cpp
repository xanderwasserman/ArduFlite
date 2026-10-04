/**
 * Bmm350.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/mag/Bmm350.h"

namespace arduflite::drivers {

namespace {

/// Sign-extend an n-bit two's-complement value held in a wider unsigned.
constexpr float fixSign(std::uint32_t value, int bits)
{
    const std::uint32_t signBit = 1u << (bits - 1);
    const std::uint32_t span    = 1u << bits;
    return (value & signBit) ? (static_cast<float>(value) - static_cast<float>(span))
                             : static_cast<float>(value);
}

/// 24-bit little-endian, XLSB first, sign-extended.
constexpr float le24s(const std::uint8_t* p)
{
    const std::uint32_t raw = static_cast<std::uint32_t>(p[0]) |
                              (static_cast<std::uint32_t>(p[1]) << 8) |
                              (static_cast<std::uint32_t>(p[2]) << 16);
    return fixSign(raw, 24);
}

// OTP word indices. Several coefficients share a word, taking either its low or
// high byte — hence the same index appearing more than once.
constexpr std::uint8_t kOtpTempOffSens = 0x0D;
constexpr std::uint8_t kOtpMagOffsetX  = 0x0E;
constexpr std::uint8_t kOtpMagOffsetY  = 0x0F;
constexpr std::uint8_t kOtpMagOffsetZ  = 0x10;
constexpr std::uint8_t kOtpMagSensX    = 0x10;
constexpr std::uint8_t kOtpMagSensY    = 0x11;
constexpr std::uint8_t kOtpMagSensZ    = 0x11;
constexpr std::uint8_t kOtpMagTcoX     = 0x12;
constexpr std::uint8_t kOtpMagTcoY     = 0x13;
constexpr std::uint8_t kOtpMagTcoZ     = 0x14;
constexpr std::uint8_t kOtpMagTcsX     = 0x12;
constexpr std::uint8_t kOtpMagTcsY     = 0x13;
constexpr std::uint8_t kOtpMagTcsZ     = 0x14;
constexpr std::uint8_t kOtpCrossXY     = 0x15;
constexpr std::uint8_t kOtpCrossYX     = 0x15;
constexpr std::uint8_t kOtpCrossZX     = 0x16;
constexpr std::uint8_t kOtpCrossZY     = 0x16;
constexpr std::uint8_t kOtpMagDutT0    = 0x18;

constexpr std::uint16_t kLsbMask = 0x00FF;
constexpr std::uint16_t kMsbMask = 0xFF00;

constexpr float kSensCorrectionY = 0.01f;
constexpr float kTcsCorrectionZ  = 0.0001f;

} // namespace

Status Bmm350::probe()
{
    std::uint8_t raw[kI2cDummyBytes + 1]{};
    if (_dev.readRegs(kRegChipId, raw, sizeof(raw)) != Status::Ok)
    {
        _health = device::SensorHealth::NotPresent;
        return Status::NotPresent;
    }

    _chipId = raw[kI2cDummyBytes];
    if (_chipId != kChipId)
    {
        _health = device::SensorHealth::Degraded;
        return Status::NotPresent;
    }

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmm350::readOtpWord(std::uint8_t address, std::uint16_t& out)
{
    using namespace std::chrono_literals;

    const std::uint8_t command =
        static_cast<std::uint8_t>(kOtpCmdDirRead | (address & kOtpWordAddrMask));
    ARDUFLITE_TRY(_dev.writeReg(kRegOtpCmd, command));

    // Poll for completion. Bounded: a part that never finishes must fail rather
    // than hang the boot, and there are 32 of these reads.
    constexpr int kMaxPolls = 20;
    for (int attempt = 0; attempt < kMaxPolls; ++attempt)
    {
        _scheduler.sleepFor(1ms);   // Bosch polls at 300 us; 1 ms is the finest we express

        std::uint8_t status[kI2cDummyBytes + 1]{};
        ARDUFLITE_TRY(_dev.readRegs(kRegOtpStatus, status, sizeof(status)));

        const std::uint8_t value = status[kI2cDummyBytes];

        // Bits 5..1 carry an error code; any of them set means the read failed
        // and polling further will not help.
        if ((value & 0x3E) != 0) { return Status::IoError; }

        if ((value & kOtpStatusCmdDone) != 0)
        {
            std::uint8_t msb[kI2cDummyBytes + 1]{};
            std::uint8_t lsb[kI2cDummyBytes + 1]{};
            ARDUFLITE_TRY(_dev.readRegs(kRegOtpDataMsb, msb, sizeof(msb)));
            ARDUFLITE_TRY(_dev.readRegs(kRegOtpDataLsb, lsb, sizeof(lsb)));

            out = static_cast<std::uint16_t>((msb[kI2cDummyBytes] << 8) | lsb[kI2cDummyBytes]);
            return Status::Ok;
        }
    }

    return Status::Timeout;
}

Status Bmm350::dumpOtp()
{
    std::uint16_t otp[kOtpWordCount]{};
    for (std::size_t i = 0; i < kOtpWordCount; ++i)
    {
        ARDUFLITE_TRY(readOtpWord(static_cast<std::uint8_t>(i), otp[i]));
    }

    unpackCompensation(otp);

    // Power the OTP down once read. It is only needed at boot, and leaving it
    // powered costs current on a battery-powered aircraft.
    return _dev.writeReg(kRegOtpCmd, kOtpCmdPowerOff);
}

void Bmm350::unpackCompensation(const std::uint16_t* otp)
{
    // Offsets are 12-bit, packed across word boundaries.
    const std::uint16_t offX = otp[kOtpMagOffsetX] & 0x0FFF;
    const std::uint16_t offY = static_cast<std::uint16_t>(((otp[kOtpMagOffsetX] & 0xF000) >> 4) +
                                                          (otp[kOtpMagOffsetY] & kLsbMask));
    const std::uint16_t offZ = static_cast<std::uint16_t>((otp[kOtpMagOffsetY] & 0x0F00) +
                                                          (otp[kOtpMagOffsetZ] & kLsbMask));

    _comp.offsetX    = fixSign(offX, 12);
    _comp.offsetY    = fixSign(offY, 12);
    _comp.offsetZ    = fixSign(offZ, 12);
    _comp.tempOffset = fixSign(otp[kOtpTempOffSens] & kLsbMask, 8) / 5.0f;

    _comp.sensX    = fixSign((otp[kOtpMagSensX] & kMsbMask) >> 8, 8) / 256.0f;
    _comp.sensY    = fixSign(otp[kOtpMagSensY] & kLsbMask, 8) / 256.0f + kSensCorrectionY;
    _comp.sensZ    = fixSign((otp[kOtpMagSensZ] & kMsbMask) >> 8, 8) / 256.0f;
    _comp.tempSens = fixSign((otp[kOtpTempOffSens] & kMsbMask) >> 8, 8) / 512.0f;

    _comp.tcoX = fixSign(otp[kOtpMagTcoX] & kLsbMask, 8) / 32.0f;
    _comp.tcoY = fixSign(otp[kOtpMagTcoY] & kLsbMask, 8) / 32.0f;
    _comp.tcoZ = fixSign(otp[kOtpMagTcoZ] & kLsbMask, 8) / 32.0f;

    _comp.tcsX = fixSign((otp[kOtpMagTcsX] & kMsbMask) >> 8, 8) / 16384.0f;
    _comp.tcsY = fixSign((otp[kOtpMagTcsY] & kMsbMask) >> 8, 8) / 16384.0f;
    _comp.tcsZ = fixSign((otp[kOtpMagTcsZ] & kMsbMask) >> 8, 8) / 16384.0f - kTcsCorrectionZ;

    _comp.t0 = fixSign(otp[kOtpMagDutT0], 16) / 512.0f + 23.0f;

    _comp.crossXY = fixSign(otp[kOtpCrossXY] & kLsbMask, 8) / 800.0f;
    _comp.crossYX = fixSign((otp[kOtpCrossYX] & kMsbMask) >> 8, 8) / 800.0f;
    _comp.crossZX = fixSign(otp[kOtpCrossZX] & kLsbMask, 8) / 800.0f;
    _comp.crossZY = fixSign((otp[kOtpCrossZY] & kMsbMask) >> 8, 8) / 800.0f;
}

Status Bmm350::begin()
{
    using namespace std::chrono_literals;

    _scheduler.sleepFor(3ms);                              // start-up from power-on
    ARDUFLITE_TRY(_dev.writeReg(kRegCmd, kCmdSoftReset));
    _scheduler.sleepFor(24ms);                             // Bosch's soft-reset delay

    // The trimming must be read before any measurement is meaningful — unlike
    // the BMP581, this part applies none of it itself.
    ARDUFLITE_TRY(dumpOtp());

    // Magnetic reset (bit-reset then flux-guide) degausses the sensor. Skipping
    // it leaves a remanent offset from whatever field the part last saw, which
    // on an airframe means the last time a motor ran.
    ARDUFLITE_TRY(_dev.writeReg(kRegPmuCmd, 0x07));        // BR
    _scheduler.sleepFor(14ms);
    ARDUFLITE_TRY(_dev.writeReg(kRegPmuCmd, 0x05));        // FGR
    _scheduler.sleepFor(18ms);

    ARDUFLITE_TRY(_dev.writeReg(kRegPmuCmdAxisEn, kEnableXyz));

    const std::uint8_t aggr =
        static_cast<std::uint8_t>(kOdr100Hz | (kAveraging4 << kAvgPos));
    ARDUFLITE_TRY(_dev.writeReg(kRegPmuCmdAggr, aggr));

    ARDUFLITE_TRY(_dev.writeReg(kRegPmuCmd, kPmuCmdNormal));
    _scheduler.sleepFor(40ms);

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmm350::sample()
{
    // 12 bytes from 0x31: X, Y, Z, then temperature — each 24-bit little-endian.
    std::uint8_t raw[kI2cDummyBytes + 12]{};
    const Status status = _dev.readRegs(kRegMagXXlsb, raw, sizeof(raw));
    if (status != Status::Ok)
    {
        _health = device::SensorHealth::Failed;
        return status;
    }

    const std::uint8_t* data = raw + kI2cDummyBytes;

    float x = le24s(data + 0) * kLsbToUtXy;
    float y = le24s(data + 3) * kLsbToUtXy;
    float z = le24s(data + 6) * kLsbToUtZ;
    float t = le24s(data + 9) * kLsbToDegC;

    // Temperature first — every magnetic term below is corrected against it.
    t = (1.0f + _comp.tempSens) * t + _comp.tempOffset;

    const float dt = t - _comp.t0;
    const float axes[3]   = { x, y, z };
    const float sens[3]   = { _comp.sensX, _comp.sensY, _comp.sensZ };
    const float tco[3]    = { _comp.tcoX,  _comp.tcoY,  _comp.tcoZ  };
    const float tcs[3]    = { _comp.tcsX,  _comp.tcsY,  _comp.tcsZ  };
    const float offset[3] = { _comp.offsetX, _comp.offsetY, _comp.offsetZ };

    float out[3];
    for (int i = 0; i < 3; ++i)
    {
        out[i]  = axes[i] * (1.0f + sens[i]);
        out[i] += offset[i];
        out[i] += tco[i] * dt;
        out[i] /= 1.0f + tcs[i] * dt;
    }

    // Cross-axis correction: each axis picks up a little of the others, and the
    // shared denominator inverts the 2x2 XY coupling.
    const float denom = 1.0f - _comp.crossYX * _comp.crossXY;

    const float compX = (out[0] - _comp.crossXY * out[1]) / denom;
    const float compY = (out[1] - _comp.crossYX * out[0]) / denom;
    const float compZ = (out[2] +
                         out[0] * (_comp.crossYX * _comp.crossZY - _comp.crossZX) -
                         out[1] * (_comp.crossZY - _comp.crossXY * _comp.crossZX)) / denom;

    _mag.field_ut = { compX, compY, compZ };
    _mag.time     = _clock.now();
    _tempC        = t;

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmm350::read(device::MagSample& out) const
{
    out = _mag;
    return Status::Ok;
}

} // namespace arduflite::drivers
