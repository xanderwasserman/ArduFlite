/**
 * Bmi323.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Bosch BMI323 6-axis IMU over hal::RegisterDevice.
 *
 * Fitted on the DFRobot SEN0697 10-DOF module alongside a BMM350 magnetometer
 * and a BMP581 barometer.
 *
 * Implements Sensor + Accelerometer + Gyroscope, exactly as Mpu6500 does — so
 * swapping one for the other is a board-descriptor change (ADR-019).
 *
 * @warning TWO THINGS make this chip unlike the MPU-6500, and both silently
 *          corrupt every sample if missed:
 *
 *   1. **Registers are 16 bits wide**, little-endian. Register 0x03 is the
 *      whole of accel X, not its high byte.
 *
 *   2. **Every I2C read returns TWO dummy bytes first.** Not one — SPI uses
 *      one, I2C uses two. Confirmed from Bosch's own driver
 *      (bmi3.c: `dev->dummy_byte = 2` for BMI3_I2C_INTF). Reading without
 *      discarding them shifts the whole burst by two bytes, which yields
 *      plausible-looking numbers rather than an obvious failure.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_IMU_BMI323_H
#define ARDUFLITE_HAL_DRIVERS_IMU_BMI323_H

#include "src/hal/core/Result.h"
#include "src/hal/device/Sensor.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/platform/RegisterDevice.h"
#include "src/hal/platform/Scheduler.h"

namespace arduflite::drivers {

class Bmi323 final : public device::Sensor,
                     public device::Accelerometer,
                     public device::Gyroscope
{
public:
    // ── Register map (Bosch BMI323 datasheet / bmi3_defs.h) ─────────────────
    static constexpr std::uint8_t kRegChipId     = 0x00;
    static constexpr std::uint8_t kRegStatus     = 0x02;
    static constexpr std::uint8_t kRegAccDataX   = 0x03;   ///< 0x03..0x05
    static constexpr std::uint8_t kRegGyrDataX   = 0x06;   ///< 0x06..0x08
    static constexpr std::uint8_t kRegTempData   = 0x09;
    static constexpr std::uint8_t kRegAccConf    = 0x20;
    static constexpr std::uint8_t kRegGyrConf    = 0x21;
    static constexpr std::uint8_t kRegCmd        = 0x7E;

    static constexpr std::uint16_t kChipId         = 0x0043;
    static constexpr std::uint16_t kCmdSoftReset   = 0xDEAF;

    /// Dummy bytes prepended to every I2C read. See the class warning.
    static constexpr std::size_t kI2cDummyBytes = 2;

    /// ACC_CONF / GYR_CONF field positions. Identical layout for both.
    static constexpr std::uint16_t kOdrMask   = 0x000F;
    static constexpr std::uint16_t kRangeMask = 0x0070;
    static constexpr std::uint8_t  kRangePos  = 4;
    static constexpr std::uint16_t kModeMask  = 0x7000;
    static constexpr std::uint8_t  kModePos   = 12;

    static constexpr std::uint8_t kOdr100Hz  = 0x08;
    static constexpr std::uint8_t kOdr200Hz  = 0x09;
    static constexpr std::uint8_t kOdr400Hz  = 0x0A;
    static constexpr std::uint8_t kOdr800Hz  = 0x0B;

    static constexpr std::uint8_t kModeHighPerformance = 0x07;

    Bmi323(hal::RegisterDevice& dev, const hal::Clock& clock, hal::Scheduler& scheduler)
        : _dev(dev), _clock(clock), _scheduler(scheduler) {}

    // ── Sensor ──────────────────────────────────────────────────────────────
    Status probe() override;
    Status begin() override;
    Status sample() override;

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return _odr_hz; }
    [[nodiscard]] const char*   name() const override { return "BMI323"; }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }

    // ── Accelerometer ───────────────────────────────────────────────────────
    Status read(device::AccelSample& out) const override;
    Status setRange_g(std::uint8_t g) override;
    [[nodiscard]] std::uint8_t range_g() const override { return _accelRange_g; }

    // ── Gyroscope ───────────────────────────────────────────────────────────
    Status read(device::GyroSample& out) const override;
    Status setRange_dps(std::uint16_t dps) override;
    [[nodiscard]] std::uint16_t range_dps() const override { return _gyroRange_dps; }

    /// The chip ID actually read. Logged on mismatch, as for the MPU-6500.
    [[nodiscard]] std::uint16_t chipId() const { return _chipId; }
    [[nodiscard]] float temperature_c() const { return _tempC; }

private:
    /// One 16-bit register, discarding the I2C dummy prefix.
    Status readWord(std::uint8_t reg, std::uint16_t& out) const;
    Status writeWord(std::uint8_t reg, std::uint16_t value);
    Status updateBits(std::uint8_t reg, std::uint16_t mask, std::uint16_t value);

    hal::RegisterDevice& _dev;
    const hal::Clock&    _clock;
    hal::Scheduler&      _scheduler;

    device::AccelSample _accel{};
    device::GyroSample  _gyro{};
    float               _tempC = 0.0f;

    std::uint16_t _chipId        = 0;
    std::uint8_t  _accelRange_g  = 4;
    std::uint16_t _gyroRange_dps = 500;
    std::uint16_t _odr_hz        = 400;

    float _accelScale = 4.0f   / 32768.0f;
    float _gyroScale  = 500.0f / 32768.0f;

    device::SensorHealth _health = device::SensorHealth::Unknown;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_IMU_BMI323_H
