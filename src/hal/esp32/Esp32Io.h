/**
 * Esp32Io.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_HAL_ESP32_IO_H
#define ARDUFLITE_HAL_ESP32_IO_H

#include "src/hal/platform/Io.h"

namespace arduflite::hal::esp32 {

class Esp32GpioPin final : public GpioPin
{
public:
    constexpr Esp32GpioPin() noexcept = default;

    void bind(std::int8_t pin) noexcept { _pin = pin; }

    Status setMode(PinMode mode) override;
    [[nodiscard]] bool read() const override;
    void write(bool high) override;

private:
    std::int8_t _pin = -1;
};

/**
 * @brief Servo/ESC PWM over the Arduino-ESP32 core's LEDC API.
 *
 * ESP32Servo is deliberately NOT used: it is ~1290 lines of per-chip #ifdef in a
 * dependency we do not control, it uses double-precision pow() at setup on a chip
 * with no FPU, and AGENTS.md rule 7 favours the core's own API over a third-party
 * wrapper. See specs/hal ADR-018.
 *
 * THIS IS THE ONLY FILE IN THE TREE THAT CALLS ledc*().
 */
class Esp32PwmOut final : public PwmOut
{
public:
    constexpr Esp32PwmOut() noexcept = default;

    void bind(std::int8_t pin) noexcept { _pin = pin; }

    Status attach(std::uint16_t minUs, std::uint16_t maxUs,
                  std::uint16_t frameRate_hz = 50) override;

    void writeMicroseconds(std::uint16_t us) override;
    void idle() override;
    void detach() override;

    [[nodiscard]] std::uint16_t lastMicroseconds() const override { return _lastUs; }

private:
    static constexpr std::uint8_t kResolutionBits = 14;   ///< 16.4 ns at 50 Hz

    std::int8_t   _pin       = -1;
    std::uint16_t _minUs     = 1000;
    std::uint16_t _maxUs     = 2000;
    std::uint16_t _frameHz   = 50;
    std::uint16_t _lastUs    = 0;
    bool          _attached  = false;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_IO_H
