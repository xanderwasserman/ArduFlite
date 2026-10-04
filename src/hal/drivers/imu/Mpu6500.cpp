/**
 * Mpu6500.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/imu/Mpu6500.h"

namespace arduflite::drivers {

namespace {

/// Range index -> register field, shifted into bits [4:3].
constexpr std::uint8_t rangeBits(std::uint8_t index) { return static_cast<std::uint8_t>(index << 3); }

/// Big-endian 16-bit, as the MPU emits.
constexpr std::int16_t be16(const std::uint8_t* p)
{
    return static_cast<std::int16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

} // namespace

Status Mpu6500::updateBits(std::uint8_t reg, std::uint8_t mask, std::uint8_t value)
{
    std::uint8_t current = 0;
    ARDUFLITE_TRY(_dev.readReg(reg, current));
    const std::uint8_t updated =
        static_cast<std::uint8_t>((current & static_cast<std::uint8_t>(~mask)) | (value & mask));
    return _dev.writeReg(reg, updated);
}

Status Mpu6500::probe()
{
    const Status s = _dev.readReg(kRegWhoAmI, _whoAmI);
    if (s != Status::Ok)
    {
        _health = device::SensorHealth::NotPresent;
        return Status::NotPresent;
    }

    if (_whoAmI != kWhoAmIExpected)
    {
        // Deliberately NOT fatal on its own. Counterfeit MPU-6500 modules are
        // common and report other IDs while behaving correctly; the caller logs
        // whoAmI() so a mismatch is diagnosable instead of a bare failure.
        _health = device::SensorHealth::Degraded;
        return Status::NotPresent;
    }

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Mpu6500::begin()
{
    // Reset, wake, then select the gyro PLL as clock source.
    //
    // The delays are NOT optional. A soft reset reloads the whole register file
    // from internal defaults, and writes issued while that is in progress are
    // dropped — leaving the IMU partly configured, usually on the wrong range,
    // so every reading is off by a constant factor. It is load- and
    // temperature-dependent, so it reproduces intermittently at best.
    using namespace std::chrono_literals;

    ARDUFLITE_TRY(_dev.writeReg(kRegPwrMgmt1, 0x80));   // device reset
    _scheduler.sleepFor(100ms);

    ARDUFLITE_TRY(_dev.writeReg(kRegPwrMgmt1, 0x00));   // wake, all sensors on
    _scheduler.sleepFor(100ms);                          // registers settling

    ARDUFLITE_TRY(_dev.writeReg(kRegPwrMgmt1, 0x01));   // clock = gyro X PLL
    _scheduler.sleepFor(200ms);                          // PLL lock

    ARDUFLITE_TRY(_dev.writeReg(kRegPwrMgmt2, 0x00));   // enable accel + gyro

    // Sample rate divider: 1000/(1+div) Hz. At the default 333 Hz the sampling
    // task reads faster than the device updates, so some reads repeat the
    // previous conversion.
    const std::uint8_t div = static_cast<std::uint8_t>(1000u / _odr_hz - 1u);
    ARDUFLITE_TRY(_dev.writeReg(kRegSmplrtDiv, div));

    // Digital low-pass filters:
    //   gyro  42 Hz -> DLPF_CFG 3 in CONFIG[2:0], FCHOICE_B cleared in GYRO_CONFIG[1:0]
    //   accel 41 Hz -> A_DLPF_CFG 3 in ACCEL_CONFIG2[3:0]
    ARDUFLITE_TRY(updateBits(kRegGyroConfig,   0x03, 0x00));
    ARDUFLITE_TRY(updateBits(kRegConfig,       0x07, 0x03));
    ARDUFLITE_TRY(updateBits(kRegAccelConfig2, 0x0F, 0x03));

    ARDUFLITE_TRY(setRange_dps(_gyroRange_dps));
    ARDUFLITE_TRY(setRange_g(_accelRange_g));

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Mpu6500::sample()
{
    // ONE burst: accel XYZ (6) + temperature (2) + gyro XYZ (6) = 14 bytes from
    // ACCEL_XOUT_H. This is why sample() and read() are separate — per-measurement
    // read() calls that each hit the bus would double I2C traffic at 500 Hz.
    std::uint8_t raw[14]{};
    const Status s = _dev.readRegs(kRegAccelXoutH, raw, sizeof(raw));
    if (s != Status::Ok)
    {
        _health = device::SensorHealth::Failed;
        return s;
    }

    const auto now = _clock.now();

    // Full-scale range over a signed 16-bit reading: value * range / 32768.
    _accel.accel_g = { be16(raw + 0) * _accelScale,
                       be16(raw + 2) * _accelScale,
                       be16(raw + 4) * _accelScale };
    _accel.time = now;

    // Datasheet: degC = raw/333.87 + 21.0
    _tempC = static_cast<float>(be16(raw + 6)) / 333.87f + 21.0f;

    _gyro.rate_dps = { be16(raw + 8)  * _gyroScale,
                       be16(raw + 10) * _gyroScale,
                       be16(raw + 12) * _gyroScale };
    _gyro.time = now;

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Mpu6500::read(device::AccelSample& out) const
{
    out = _accel;   // cached; never touches the bus
    return Status::Ok;
}

Status Mpu6500::read(device::GyroSample& out) const
{
    out = _gyro;
    return Status::Ok;
}

Status Mpu6500::setRange_g(std::uint8_t g)
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
    return updateBits(kRegAccelConfig, 0x18, rangeBits(index));
}

Status Mpu6500::setRange_dps(std::uint16_t dps)
{
    std::uint8_t index = 0;
    switch (dps)
    {
        case 250:  index = 0; break;
        case 500:  index = 1; break;
        case 1000: index = 2; break;
        case 2000: index = 3; break;
        default: return Status::InvalidArg;
    }
    _gyroRange_dps = dps;
    _gyroScale     = static_cast<float>(dps) / 32768.0f;
    return updateBits(kRegGyroConfig, 0x18, rangeBits(index));
}

} // namespace arduflite::drivers
