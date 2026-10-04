/**
 * Bmp280.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/baro/Bmp280.h"

namespace arduflite::drivers {

namespace {

/// Calibration words are little-endian, unlike the sensor data.
constexpr std::uint16_t le16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}

constexpr std::int16_t le16s(const std::uint8_t* p)
{
    return static_cast<std::int16_t>(le16(p));
}

/// Sensor readings are 20-bit, big-endian, left-aligned across three bytes.
constexpr std::int32_t be20(const std::uint8_t* p)
{
    return static_cast<std::int32_t>((static_cast<std::uint32_t>(p[0]) << 12) |
                                     (static_cast<std::uint32_t>(p[1]) << 4)  |
                                     (static_cast<std::uint32_t>(p[2]) >> 4));
}

} // namespace

std::int32_t Bmp280::compensateTemperature(const Calibration& cal,
                                           std::int32_t rawTemperature,
                                           std::int32_t& fineTemperature)
{
    // Bosch datasheet BST-BMP280-DS001, bmp280_compensate_T_int32().
    const std::int32_t var1 =
        (((rawTemperature >> 3) - (static_cast<std::int32_t>(cal.t1) << 1)) *
         static_cast<std::int32_t>(cal.t2)) >> 11;

    const std::int32_t var2 =
        ((((rawTemperature >> 4) - static_cast<std::int32_t>(cal.t1)) *
          ((rawTemperature >> 4) - static_cast<std::int32_t>(cal.t1))) >> 12) *
        static_cast<std::int32_t>(cal.t3) >> 14;

    // t_fine carries the temperature into the pressure compensation. Pressure is
    // meaningless without it, which is why sample() always converts both.
    fineTemperature = var1 + var2;
    return (fineTemperature * 5 + 128) >> 8;
}

std::uint32_t Bmp280::compensatePressure(const Calibration& cal,
                                         std::int32_t rawPressure,
                                         std::int32_t fineTemperature)
{
    // Bosch datasheet, bmp280_compensate_P_int64(). 64-bit throughout: the 32-bit
    // variant loses ~1 Pa, which at sea level is ~8 cm of altitude.
    std::int64_t var1 = static_cast<std::int64_t>(fineTemperature) - 128000;
    std::int64_t var2 = var1 * var1 * static_cast<std::int64_t>(cal.p6);
    var2 = var2 + ((var1 * static_cast<std::int64_t>(cal.p5)) << 17);
    var2 = var2 + (static_cast<std::int64_t>(cal.p4) << 35);
    var1 = ((var1 * var1 * static_cast<std::int64_t>(cal.p3)) >> 8) +
           ((var1 * static_cast<std::int64_t>(cal.p2)) << 12);
    var1 = (((static_cast<std::int64_t>(1) << 47) + var1) *
            static_cast<std::int64_t>(cal.p1)) >> 33;

    if (var1 == 0)
    {
        return 0;   // uncalibrated part; division would trap
    }

    std::int64_t p = 1048576 - rawPressure;
    p = (((p << 31) - var2) / var1) * 3125;
    var1 = (static_cast<std::int64_t>(cal.p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (static_cast<std::int64_t>(cal.p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (static_cast<std::int64_t>(cal.p7) << 4);

    return static_cast<std::uint32_t>(p);   // Q24.8 Pa
}

Status Bmp280::probe()
{
    const Status s = _dev.readReg(kRegId, _chipId);
    if (s != Status::Ok)
    {
        _health = device::SensorHealth::NotPresent;
        return Status::NotPresent;
    }

    if (_chipId != kIdExpected)
    {
        // 0x60 is a BME280 (same registers plus humidity) and 0x56/0x57 are
        // BMP280 samples. Reporting the byte lets the log say which.
        _health = device::SensorHealth::Degraded;
        return Status::NotPresent;
    }

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmp280::begin()
{
    using namespace std::chrono_literals;

    // Resetting means a warm reboot cannot inherit the previous run's
    // configuration, but it brings an obligation: the part copies its factory
    // trimming from NVM into the image registers during startup, and the
    // calibration block reads back garbage until that finishes.
    //
    // Reading it early does not fail — it returns plausible-looking numbers that
    // are simply wrong, and every pressure the aircraft ever computes is then
    // wrong by an amount no bench check would flag as obviously broken. So poll
    // the status bit rather than guessing at a delay.
    ARDUFLITE_TRY(_dev.writeReg(kRegReset, kResetCommand));
    _scheduler.sleepFor(2ms);   // datasheet t_startup, before STATUS is meaningful

    constexpr int kMaxAttempts = 25;   // 25 x 2 ms = 50 ms, ~25x the typical wait
    int attempts = 0;
    for (;;)
    {
        std::uint8_t status = 0;
        ARDUFLITE_TRY(_dev.readReg(kRegStatus, status));
        if ((status & kStatusImUpdate) == 0) { break; }

        if (++attempts >= kMaxAttempts)
        {
            // Stuck asserted means the part is not completing startup. Refusing
            // here is the whole point: the alternative is a barometer that reads
            // confidently and wrongly for the entire flight.
            _health = device::SensorHealth::Failed;
            return Status::Timeout;
        }
        _scheduler.sleepFor(2ms);
    }

    std::uint8_t raw[24]{};
    ARDUFLITE_TRY(_dev.readRegs(kRegCalibration, raw, sizeof(raw)));

    _cal.t1 = le16 (raw + 0);
    _cal.t2 = le16s(raw + 2);
    _cal.t3 = le16s(raw + 4);
    _cal.p1 = le16 (raw + 6);
    _cal.p2 = le16s(raw + 8);
    _cal.p3 = le16s(raw + 10);
    _cal.p4 = le16s(raw + 12);
    _cal.p5 = le16s(raw + 14);
    _cal.p6 = le16s(raw + 16);
    _cal.p7 = le16s(raw + 18);
    _cal.p8 = le16s(raw + 20);
    _cal.p9 = le16s(raw + 22);

    // An all-zero or all-0xFF calibration block means the read succeeded but the
    // part did not answer meaningfully. Compensation would silently produce
    // garbage pressure, so refuse here instead.
    if (_cal.t1 == 0 || _cal.p1 == 0)
    {
        _health = device::SensorHealth::Failed;
        return Status::IoError;
    }

    // Pinned as byte assertions in test_bmp280.cpp: these are what the aircraft
    // flies with, and changing one silently changes filtering or oversampling.
    //
    //   CONFIG:    t_sb = 0 (0.5 ms standby), filter = OFF
    //   CTRL_MEAS: osrs_t = x16, osrs_p = x16, mode = normal
    //
    // Turning the hardware IIR filter ON is tempting — it would suppress the
    // pressure spikes a canopy and propeller wash cause — but it is NOT done
    // here. AltitudeFilter already runs its own altitude EMA (altiAlpha) at the
    // baro rate and differentiates it for climb rate; adding a second filter in
    // series would change the vario's dynamics. That is a tuning decision to
    // make deliberately, against a flying aircraft, not a side effect of
    // swapping libraries.
    ARDUFLITE_TRY(_dev.writeReg(kRegConfig,
        static_cast<std::uint8_t>(static_cast<std::uint8_t>(FilterCoefficient::Off) << 2)));

    ARDUFLITE_TRY(_dev.writeReg(kRegCtrlMeas,
        static_cast<std::uint8_t>((static_cast<std::uint8_t>(Oversampling::X16) << 5) |
                                  (static_cast<std::uint8_t>(Oversampling::X16) << 2) |
                                  0x03)));

    // Normal mode starts converting immediately, but the first result is not
    // ready for one measurement period.
    _scheduler.sleepFor(100ms);

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmp280::sample()
{
    // One 6-byte burst covering 0xF7..0xFC. Reading pressure and temperature in
    // separate transactions would let a conversion land between them, pairing a
    // pressure with the t_fine of a different measurement.
    std::uint8_t raw[6]{};
    const Status s = _dev.readRegs(kRegPressMsb, raw, sizeof(raw));
    if (s != Status::Ok)
    {
        _health = device::SensorHealth::Failed;
        return s;
    }

    const std::int32_t rawPressure    = be20(raw + 0);
    const std::int32_t rawTemperature = be20(raw + 3);

    std::int32_t fine = 0;
    const std::int32_t t_centi = compensateTemperature(_cal, rawTemperature, fine);
    const std::uint32_t p_q248 = compensatePressure(_cal, rawPressure, fine);

    _baro.pressure_pa   = static_cast<float>(p_q248) / 256.0f;
    _baro.temp_c = static_cast<float>(t_centi) / 100.0f;
    _baro.time          = _clock.now();

    _health = device::SensorHealth::Ok;
    return Status::Ok;
}

Status Bmp280::read(device::BaroSample& out) const
{
    out = _baro;
    return Status::Ok;
}

} // namespace arduflite::drivers
