/**
 * Mpu6500.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief InvenSense MPU-6500 over hal::RegisterDevice.
 *
 * Implements THREE interfaces: Sensor (lifecycle + sample), Accelerometer
 * and Gyroscope. One chip, several measurements — ADR-019.
 *
 * Going through RegisterDevice rather than a bus type directly is what lets
 * this be tested on a host and moved to SPI without touching the driver
 * (ADR-013). Register writes and scaling are pinned in test_mpu6500.cpp.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_IMU_MPU6500_H
#define ARDUFLITE_HAL_DRIVERS_IMU_MPU6500_H

#include "src/hal/core/Result.h"
#include "src/hal/device/Sensor.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/platform/Scheduler.h"
#include "src/hal/platform/RegisterDevice.h"

namespace arduflite::drivers {

class Mpu6500 : public device::Sensor,
                public device::Accelerometer,
                public device::Gyroscope
{
public:
    // ── Register map (datasheet names) ──────────────────────────────────────
    static constexpr std::uint8_t kRegSmplrtDiv    = 0x19;
    static constexpr std::uint8_t kRegConfig       = 0x1A;
    static constexpr std::uint8_t kRegGyroConfig   = 0x1B;
    static constexpr std::uint8_t kRegAccelConfig  = 0x1C;
    static constexpr std::uint8_t kRegAccelConfig2 = 0x1D;
    static constexpr std::uint8_t kRegAccelXoutH   = 0x3B;
    static constexpr std::uint8_t kRegPwrMgmt1     = 0x6B;
    static constexpr std::uint8_t kRegPwrMgmt2     = 0x6C;
    static constexpr std::uint8_t kRegWhoAmI       = 0x75;

    /// Genuine MPU-6500. Clones report other values — see probe().
    static constexpr std::uint8_t kWhoAmIExpected  = 0x70;

    /// @param scheduler needed only by begin(): the power-up sequence has
    ///        mandatory settling delays between register writes (see begin()).
    ///        sample() and read() never touch it.
    Mpu6500(hal::RegisterDevice& dev, const hal::Clock& clock, hal::Scheduler& scheduler)
        : _dev(dev), _clock(clock), _scheduler(scheduler) {}

    // ── Sensor ──────────────────────────────────────────────────────────────
    Status probe() override;
    Status begin() override;
    Status sample() override;

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return _odr_hz; }
    [[nodiscard]] const char*   name()   const override { return "MPU-6500"; }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }

    // ── Accelerometer ───────────────────────────────────────────────────────
    Status read(device::AccelSample& out) const override;
    Status setRange_g(std::uint8_t g) override;
    [[nodiscard]] std::uint8_t range_g() const override { return _accelRange_g; }

    // ── Gyroscope ───────────────────────────────────────────────────────────
    Status read(device::GyroSample& out) const override;
    Status setRange_dps(std::uint16_t dps) override;
    [[nodiscard]] std::uint16_t range_dps() const override { return _gyroRange_dps; }

    /// The byte WHO_AM_I actually returned, kept rather than reduced to a
    /// pass/fail: on a suspected counterfeit part it is the whole diagnosis.
    [[nodiscard]] std::uint8_t whoAmI() const { return _whoAmI; }

    [[nodiscard]] float temperature_c() const { return _tempC; }

protected:
    /// Read-modify-write, for the range and filter fields.
    Status updateBits(std::uint8_t reg, std::uint8_t mask, std::uint8_t value);

    hal::RegisterDevice& _dev;
    const hal::Clock&    _clock;
    hal::Scheduler&      _scheduler;

    device::AccelSample _accel{};
    device::GyroSample  _gyro{};
    float               _tempC = 0.0f;

    std::uint8_t  _whoAmI        = 0;
    std::uint8_t  _accelRange_g  = 4;      ///< ArduFlite's flying value
    std::uint16_t _gyroRange_dps = 500;    ///< ArduFlite's flying value
    std::uint16_t _odr_hz        = 333;    ///< FastIMU's setGyroODR(333)

    float _accelScale = 4.0f   / 32768.0f;
    float _gyroScale  = 500.0f / 32768.0f;

    device::SensorHealth _health = device::SensorHealth::Unknown;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_IMU_MPU6500_H
