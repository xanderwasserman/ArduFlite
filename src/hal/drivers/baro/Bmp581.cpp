/**
 * Bmp581.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/baro/Bmp581.h"

namespace arduflite::drivers {

namespace {

/// 24-bit little-endian, XLSB first.
constexpr std::uint32_t le24(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16);
}

/// Same, sign-extended from 24 to 32 bits — temperature can be negative.
constexpr std::int32_t le24s(const std::uint8_t* p)
{
    const std::uint32_t raw = le24(p);
    return (raw & 0x00800000u) ? static_cast<std::int32_t>(raw | 0xFF000000u)
                               : static_cast<std::int32_t>(raw);
}

} // namespace

Status Bmp581::probe()
{
    if (_dev.readReg(kRegChipId, _chipId) != Status::Ok)
    {
        _health = device::SensorHealth::NotPresent;
        return Status::NotPresent;
    }

    if (_chipId != kChipId)
    {
        _health = device::SensorHealth::Degraded;
        return Status::NotPresent;
    }

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmp581::begin()
{
    using namespace std::chrono_literals;

    ARDUFLITE_TRY(_dev.writeReg(kRegCmd, kCmdSoftReset));

    // The part reloads its trimming from NVM during startup, exactly as the
    // BMP280 does — the difference is that it applies the trimming ITSELF, so
    // there is no coefficient block for us to read too early (ADR-030).
    _scheduler.sleepFor(5ms);

    // Pressure enabled, pressure oversampled x16 for resolution, temperature
    // x1 — temperature is only needed to compensate pressure, which the part
    // does internally, so oversampling it costs conversion time for nothing.
    const std::uint8_t osr =
        static_cast<std::uint8_t>((kOversampling1  << kOsrTempPos)  |
                                  (kOversampling16 << kOsrPressPos) |
                                  (1u              << kPressEnPos));
    ARDUFLITE_TRY(_dev.writeReg(kRegOsrConfig, osr));

    // Normal mode at 50 Hz, with deep-sleep disabled: a part that duty-cycles
    // itself hands the altitude filter repeats of the same conversion.
    const std::uint8_t odr =
        static_cast<std::uint8_t>((kModeNormal << kPwrModePos) |
                                  (kOdr50Hz    << kOdrPos)     |
                                  (1u          << kDeepDisPos));
    ARDUFLITE_TRY(_dev.writeReg(kRegOdrConfig, odr));

    // One conversion period before the first reading is meaningful.
    _scheduler.sleepFor(25ms);

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmp581::sample()
{
    // Six bytes, 0x1D..0x22: temperature THEN pressure. The BMP280 had them the
    // other way round, so a copied offset silently swaps the two.
    std::uint8_t raw[6]{};
    const Status status = _dev.readRegs(kRegTempData, raw, sizeof(raw));
    if (status != Status::Ok)
    {
        _health = device::SensorHealth::Failed;
        return status;
    }

    // Compensated by the part. No polynomial, no coefficients.
    _baro.temp_c      = static_cast<float>(le24s(raw + 0)) / 65536.0f;
    _baro.pressure_pa = static_cast<float>(le24(raw + 3))  / 64.0f;
    _baro.time        = _clock.now();

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmp581::read(device::BaroSample& out) const
{
    out = _baro;
    return Status::Ok;
}

} // namespace arduflite::drivers
