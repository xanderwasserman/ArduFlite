/**
 * Bmp581.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Bosch BMP581 barometer over hal::RegisterDevice.
 *
 * On the DFRobot SEN0697 10-DOF module, alongside a BMI323 and a BMM350.
 *
 * @note Markedly simpler than the BMP280 it can replace: the BMP581 outputs
 *       COMPENSATED values directly. There is no NVM trimming block to read,
 *       no `im_update` window to wait out, and no compensation polynomial —
 *       temperature is a signed 24-bit count over 65536 degC, pressure an
 *       unsigned 24-bit count over 64 Pa. Everything ADR-031 had to pin about
 *       the BMP280's coefficients simply does not exist here.
 *
 * @note Data is 24-bit LITTLE-endian (XLSB first), and TEMPERATURE comes
 *       before pressure in the register map — the opposite order to the BMP280.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_BARO_BMP581_H
#define ARDUFLITE_HAL_DRIVERS_BARO_BMP581_H

#include "src/hal/core/Result.h"
#include "src/hal/device/Sensor.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/platform/RegisterDevice.h"
#include "src/hal/platform/Scheduler.h"

namespace arduflite::drivers {

class Bmp581 final : public device::Sensor, public device::Barometer
{
public:
    static constexpr std::uint8_t kRegChipId    = 0x01;
    static constexpr std::uint8_t kRegTempData  = 0x1D;   ///< 0x1D..0x1F, then pressure
    static constexpr std::uint8_t kRegStatus    = 0x28;
    static constexpr std::uint8_t kRegOsrConfig = 0x36;
    static constexpr std::uint8_t kRegOdrConfig = 0x37;
    static constexpr std::uint8_t kRegCmd       = 0x7E;

    static constexpr std::uint8_t kChipId       = 0x50;
    static constexpr std::uint8_t kCmdSoftReset = 0xB6;

    /// OSR_CONFIG fields.
    static constexpr std::uint8_t kOsrTempPos  = 0;
    static constexpr std::uint8_t kOsrPressPos = 3;
    static constexpr std::uint8_t kPressEnPos  = 6;

    /// ODR_CONFIG fields.
    static constexpr std::uint8_t kPwrModePos = 0;
    static constexpr std::uint8_t kOdrPos     = 2;
    static constexpr std::uint8_t kDeepDisPos = 7;

    static constexpr std::uint8_t kModeNormal     = 1;
    static constexpr std::uint8_t kOdr50Hz        = 0x0F;
    static constexpr std::uint8_t kOversampling1  = 0;
    static constexpr std::uint8_t kOversampling16 = 4;

    Bmp581(hal::RegisterDevice& dev, const hal::Clock& clock, hal::Scheduler& scheduler)
        : _dev(dev), _clock(clock), _scheduler(scheduler) {}

    // ── Sensor ──────────────────────────────────────────────────────────────
    Status probe() override;
    Status begin() override;
    Status sample() override;

    /// Configured ODR. InertialSubsystem decimates its reads against this
    /// (ADR-046), so it must describe what the part actually produces.
    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return 50; }
    [[nodiscard]] const char*   name() const override { return "BMP581"; }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }

    // ── Barometer ───────────────────────────────────────────────────────────
    Status read(device::BaroSample& out) const override;

    [[nodiscard]] std::uint8_t chipId() const { return _chipId; }

private:
    hal::RegisterDevice& _dev;
    const hal::Clock&    _clock;
    hal::Scheduler&      _scheduler;

    device::BaroSample _baro{};
    std::uint8_t       _chipId = 0;

    device::SensorHealth _health = device::SensorHealth::Unknown;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_BARO_BMP581_H
