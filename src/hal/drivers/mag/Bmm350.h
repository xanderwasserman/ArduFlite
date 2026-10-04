/**
 * Bmm350.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Bosch BMM350 3-axis magnetometer over hal::RegisterDevice.
 *
 * On the DFRobot SEN0697 10-DOF module, alongside a BMI323 and a BMP581.
 *
 * @note Shares the BMI323's I2C quirk: every read is preceded by TWO dummy
 *       bytes (bmm350.c: `temp_len = len + BMM350_DUMMY_BYTES`, and that
 *       constant is 2).
 *
 * @note Unlike the BMP581, this part does NOT compensate itself. Factory
 *       trimming lives in OTP and has to be read out word by word through a
 *       command/status handshake, unpacked into offset, sensitivity, TCO, TCS
 *       and cross-axis terms, and applied on every sample. Those constants
 *       and divisors come from Bosch's published driver and are not derivable
 *       from the datasheet alone — check them against that source before
 *       changing any of them.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_MAG_BMM350_H
#define ARDUFLITE_HAL_DRIVERS_MAG_BMM350_H

#include "src/hal/core/Result.h"
#include "src/hal/device/Sensor.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/platform/RegisterDevice.h"
#include "src/hal/platform/Scheduler.h"

namespace arduflite::drivers {

class Bmm350 final : public device::Sensor, public device::Magnetometer
{
public:
    // ── Register map ────────────────────────────────────────────────────────
    static constexpr std::uint8_t kRegChipId       = 0x00;
    static constexpr std::uint8_t kRegPmuCmdAggr   = 0x04;
    static constexpr std::uint8_t kRegPmuCmdAxisEn = 0x05;
    static constexpr std::uint8_t kRegPmuCmd       = 0x06;
    static constexpr std::uint8_t kRegMagXXlsb     = 0x31;   ///< 0x31..0x3C incl. temp
    static constexpr std::uint8_t kRegOtpCmd       = 0x50;
    static constexpr std::uint8_t kRegOtpDataMsb   = 0x52;
    static constexpr std::uint8_t kRegOtpDataLsb   = 0x53;
    static constexpr std::uint8_t kRegOtpStatus    = 0x55;
    static constexpr std::uint8_t kRegCmd          = 0x7E;

    static constexpr std::uint8_t kChipId       = 0x33;
    static constexpr std::uint8_t kCmdSoftReset = 0xB6;

    static constexpr std::size_t kI2cDummyBytes = 2;

    /// OTP access.
    static constexpr std::uint8_t kOtpCmdDirRead    = 0x20;
    static constexpr std::uint8_t kOtpCmdPowerOff   = 0x80;
    static constexpr std::uint8_t kOtpWordAddrMask  = 0x1F;
    static constexpr std::uint8_t kOtpStatusCmdDone = 0x01;
    static constexpr std::size_t  kOtpWordCount     = 32;

    /// PMU commands and config.
    static constexpr std::uint8_t kPmuCmdNormal = 0x01;
    static constexpr std::uint8_t kEnableXyz    = 0x07;
    static constexpr std::uint8_t kOdr100Hz     = 0x04;
    static constexpr std::uint8_t kAveraging4   = 0x02;
    static constexpr std::uint8_t kAvgPos       = 4;

    Bmm350(hal::RegisterDevice& dev, const hal::Clock& clock, hal::Scheduler& scheduler)
        : _dev(dev), _clock(clock), _scheduler(scheduler) {}

    // ── Sensor ──────────────────────────────────────────────────────────────
    Status probe() override;
    Status begin() override;
    Status sample() override;

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return 100; }
    [[nodiscard]] const char*   name() const override { return "BMM350"; }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }

    // ── Magnetometer ────────────────────────────────────────────────────────
    Status read(device::MagSample& out) const override;

    [[nodiscard]] std::uint8_t chipId() const { return _chipId; }
    [[nodiscard]] float temperature_c() const { return _tempC; }

    /**
     * @brief Factory trimming, unpacked from OTP.
     *
     * Exposed so a test can inject known values and check the compensation
     * arithmetic without emulating the OTP handshake.
     */
    struct Compensation
    {
        float offsetX = 0.0f, offsetY = 0.0f, offsetZ = 0.0f, tempOffset = 0.0f;
        float sensX   = 0.0f, sensY   = 0.0f, sensZ   = 0.0f, tempSens   = 0.0f;
        float tcoX    = 0.0f, tcoY    = 0.0f, tcoZ    = 0.0f;
        float tcsX    = 0.0f, tcsY    = 0.0f, tcsZ    = 0.0f;
        float crossXY = 0.0f, crossYX = 0.0f, crossZX = 0.0f, crossZY = 0.0f;
        float t0      = 23.0f;
    };

    [[nodiscard]] const Compensation& compensation() const { return _comp; }
    void setCompensationForTest(const Compensation& c) { _comp = c; }

    /// LSB -> microtesla, from Bosch's update_default_coefiecents().
    static constexpr float kLsbToUtXy = 0.9536743164f / (14.55f * 19.46f * (1.0f / 1.5f) * 0.714607238769531f);
    static constexpr float kLsbToUtZ  = 0.9536743164f / (9.0f  * 31.0f  * (1.0f / 1.5f) * 0.714607238769531f);
    static constexpr float kLsbToDegC = 1.0f / (0.00204f * (1.0f / 1.5f) * 0.714607238769531f * 1048576.0f);

private:
    Status readOtpWord(std::uint8_t address, std::uint16_t& out);
    Status dumpOtp();
    void   unpackCompensation(const std::uint16_t* otp);

    hal::RegisterDevice& _dev;
    const hal::Clock&    _clock;
    hal::Scheduler&      _scheduler;

    device::MagSample _mag{};
    float             _tempC = 0.0f;

    std::uint8_t _chipId = 0;
    Compensation _comp{};

    device::SensorHealth _health = device::SensorHealth::Unknown;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_MAG_BMM350_H
