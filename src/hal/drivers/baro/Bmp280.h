/**
 * Bmp280.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Bosch BMP280 pressure/temperature sensor over hal::RegisterDevice.
 *
 * Register configuration is pinned as byte assertions in test_bmp280.cpp: the
 * values are what the aircraft flies with, and changing one silently changes
 * filtering or oversampling.
 *
 * The compensation arithmetic is where a silent error is invisible on the bench
 * — a wrong coefficient sign reads as a plausible altitude that drifts — so it
 * is checked against Bosch's own worked example in the same file.
 *
 * Integer compensation, not the floating-point variant: the ESP32-C3 is rv32imc
 * with a soft-float ABI (no FPU), so the float path would call __mulsf3 per term.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_BARO_BMP280_H
#define ARDUFLITE_HAL_DRIVERS_BARO_BMP280_H

#include "src/hal/core/Result.h"
#include "src/hal/device/Sensor.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/platform/Scheduler.h"
#include "src/hal/platform/RegisterDevice.h"

namespace arduflite::drivers {

class Bmp280 final : public device::Sensor,
                     public device::Barometer
{
public:
    static constexpr std::uint8_t kRegCalibration = 0x88;  ///< 0x88..0x9F, 24 bytes
    static constexpr std::uint8_t kRegId          = 0xD0;
    static constexpr std::uint8_t kRegReset       = 0xE0;
    static constexpr std::uint8_t kRegStatus      = 0xF3;
    static constexpr std::uint8_t kRegCtrlMeas    = 0xF4;
    static constexpr std::uint8_t kRegConfig      = 0xF5;
    static constexpr std::uint8_t kRegPressMsb    = 0xF7;  ///< 0xF7..0xFC, 6 bytes

    static constexpr std::uint8_t kIdExpected   = 0x58;
    static constexpr std::uint8_t kResetCommand = 0xB6;

    /// STATUS bit 0. Set while the part is copying trimming from NVM into its
    /// image registers; the calibration block reads back garbage until it clears.
    static constexpr std::uint8_t kStatusImUpdate = 0x01;

    /// Oversampling, as encoded in CTRL_MEAS. Higher costs conversion time.
    enum class Oversampling : std::uint8_t { Skip = 0, X1 = 1, X2 = 2, X4 = 3, X8 = 4, X16 = 5 };

    /// IIR filter coefficient, as encoded in CONFIG[4:2].
    enum class FilterCoefficient : std::uint8_t { Off = 0, X2 = 1, X4 = 2, X8 = 3, X16 = 4 };

    /// @param scheduler used only by begin(), to wait out the NVM copy that
    ///        follows a soft reset. sample() and read() never touch it.
    Bmp280(hal::RegisterDevice& dev, const hal::Clock& clock, hal::Scheduler& scheduler)
        : _dev(dev), _clock(clock), _scheduler(scheduler) {}

    // ── Sensor ──────────────────────────────────────────────────────────────
    Status probe() override;
    Status begin() override;
    Status sample() override;

    /// x16/x16 oversampling with the IIR filter off gives a typical measurement
    /// time near 66 ms, i.e. ~15 Hz. InertialSubsystem decimates its reads from
    /// this value, so it is the single source of truth for the barometer's rate
    /// — the climb-rate derivative divides by the interval derived from it.
    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return 15; }
    [[nodiscard]] const char*   name()   const override { return "BMP280"; }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }

    // ── Barometer ───────────────────────────────────────────────────────────
    Status read(device::BaroSample& out) const override;

    [[nodiscard]] std::uint8_t chipId() const { return _chipId; }

    /// Bosch's factory trimming, as read from 0x88. Exposed for the datasheet test.
    struct Calibration
    {
        std::uint16_t t1 = 0;
        std::int16_t  t2 = 0, t3 = 0;
        std::uint16_t p1 = 0;
        std::int16_t  p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0, p7 = 0, p8 = 0, p9 = 0;
    };

    [[nodiscard]] const Calibration& calibration() const { return _cal; }
    void setCalibrationForTest(const Calibration& c) { _cal = c; }

    /**
     * @brief Bosch's compensation, verbatim from the datasheet (§3.11.3).
     *
     * Deliberately kept as free-standing pure functions so the datasheet's worked
     * example can be run against them directly, with no device involved.
     *
     * @return temperature in 0.01 degC; pressure in Q24.8 Pa.
     */
    static std::int32_t compensateTemperature(const Calibration& cal,
                                              std::int32_t rawTemperature,
                                              std::int32_t& fineTemperature);
    static std::uint32_t compensatePressure(const Calibration& cal,
                                            std::int32_t rawPressure,
                                            std::int32_t fineTemperature);

private:
    hal::RegisterDevice& _dev;
    const hal::Clock&    _clock;
    hal::Scheduler&      _scheduler;

    Calibration        _cal{};
    device::BaroSample _baro{};
    std::uint8_t       _chipId = 0;

    device::SensorHealth _health = device::SensorHealth::Unknown;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_BARO_BMP280_H
