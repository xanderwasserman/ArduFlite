/**
 * Bmi323.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/imu/Bmi323.h"

namespace arduflite::drivers {

namespace {

/// BMI323 words are little-endian, unlike the MPU-6500's big-endian samples.
constexpr std::uint16_t le16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}

constexpr std::int16_t le16s(const std::uint8_t* p)
{
    return static_cast<std::int16_t>(le16(p));
}

} // namespace

Status Bmi323::readWord(std::uint8_t reg, std::uint16_t& out) const
{
    // Two dummy bytes, then the word. See the warning in the header.
    std::uint8_t raw[kI2cDummyBytes + 2]{};
    ARDUFLITE_TRY(_dev.readRegs(reg, raw, sizeof(raw)));

    out = le16(raw + kI2cDummyBytes);
    return Status::Ok;
}

Status Bmi323::writeWord(std::uint8_t reg, std::uint16_t value)
{
    // Writes carry NO dummy bytes — the prefix is a read-path artefact only.
    const std::uint8_t payload[2] = { static_cast<std::uint8_t>(value & 0xFF),
                                      static_cast<std::uint8_t>(value >> 8) };
    return _dev.writeRegs(reg, payload, sizeof(payload));
}

Status Bmi323::updateBits(std::uint8_t reg, std::uint16_t mask, std::uint16_t value)
{
    std::uint16_t current = 0;
    ARDUFLITE_TRY(readWord(reg, current));

    const std::uint16_t updated =
        static_cast<std::uint16_t>((current & static_cast<std::uint16_t>(~mask)) | (value & mask));
    return writeWord(reg, updated);
}

Status Bmi323::probe()
{
    if (readWord(kRegChipId, _chipId) != Status::Ok)
    {
        _health = device::SensorHealth::NotPresent;
        return Status::NotPresent;
    }

    // The ID lives in the low byte; the high byte carries a revision that
    // varies between parts, so comparing the whole word would reject valid
    // silicon.
    if ((_chipId & 0x00FF) != (kChipId & 0x00FF))
    {
        _health = device::SensorHealth::Degraded;
        return Status::NotPresent;
    }

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmi323::begin()
{
    using namespace std::chrono_literals;

    ARDUFLITE_TRY(writeWord(kRegCmd, kCmdSoftReset));

    // Bosch's driver waits 1500 us after a soft reset before touching anything.
    // Rounded up to a whole millisecond: this runs once, at boot.
    _scheduler.sleepFor(2ms);

    // The reset drops the part back to suspend mode, so the chip ID has to be
    // re-read to bring the interface up before configuration.
    std::uint16_t ignored = 0;
    (void)readWord(kRegChipId, ignored);

    // High-performance mode on both, so neither duty-cycles. A 500 Hz control
    // loop reading a duty-cycled sensor gets repeats.
    const std::uint16_t accConf =
        static_cast<std::uint16_t>(kOdr400Hz) |
        static_cast<std::uint16_t>(kModeHighPerformance << kModePos);
    const std::uint16_t gyrConf =
        static_cast<std::uint16_t>(kOdr400Hz) |
        static_cast<std::uint16_t>(kModeHighPerformance << kModePos);

    ARDUFLITE_TRY(writeWord(kRegAccConf, accConf));
    ARDUFLITE_TRY(writeWord(kRegGyrConf, gyrConf));

    // Ranges last: they read-modify-write the same registers just written.
    ARDUFLITE_TRY(setRange_g(_accelRange_g));
    ARDUFLITE_TRY(setRange_dps(_gyroRange_dps));

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmi323::sample()
{
    // One burst covering ACC_X..GYR_Z — six 16-bit registers, twelve bytes,
    // behind the two dummy bytes. Accel and gyro therefore come from the same
    // instant, which the fusion filter assumes.
    std::uint8_t raw[kI2cDummyBytes + 12]{};
    const Status status = _dev.readRegs(kRegAccDataX, raw, sizeof(raw));
    if (status != Status::Ok)
    {
        _health = device::SensorHealth::Failed;
        return status;
    }

    const std::uint8_t* data = raw + kI2cDummyBytes;
    const auto now = _clock.now();

    _accel.accel_g = { le16s(data + 0) * _accelScale,
                       le16s(data + 2) * _accelScale,
                       le16s(data + 4) * _accelScale };
    _accel.time = now;

    _gyro.rate_dps = { le16s(data + 6)  * _gyroScale,
                       le16s(data + 8)  * _gyroScale,
                       le16s(data + 10) * _gyroScale };
    _gyro.time = now;

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmi323::read(device::AccelSample& out) const { out = _accel; return Status::Ok; }
Status Bmi323::read(device::GyroSample& out)  const { out = _gyro;  return Status::Ok; }

Status Bmi323::setRange_g(std::uint8_t g)
{
    std::uint8_t index = 0;
    switch (g)
    {
        case 2:  index = 0; break;
        case 4:  index = 1; break;
        case 8:  index = 2; break;
        case 16: index = 3; break;
        default: return Status::InvalidArg;
    }

    _accelRange_g = g;
    _accelScale   = static_cast<float>(g) / 32768.0f;
    return updateBits(kRegAccConf, kRangeMask, static_cast<std::uint16_t>(index << kRangePos));
}

Status Bmi323::setRange_dps(std::uint16_t dps)
{
    std::uint8_t index = 0;
    switch (dps)
    {
        case 125:  index = 0; break;
        case 250:  index = 1; break;
        case 500:  index = 2; break;
        case 1000: index = 3; break;
        case 2000: index = 4; break;
        default: return Status::InvalidArg;
    }

    _gyroRange_dps = dps;
    _gyroScale     = static_cast<float>(dps) / 32768.0f;
    return updateBits(kRegGyrConf, kRangeMask, static_cast<std::uint16_t>(index << kRangePos));
}

} // namespace arduflite::drivers
