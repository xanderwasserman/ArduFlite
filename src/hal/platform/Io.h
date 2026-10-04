/**
 * Io.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief GPIO and PWM output.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_IO_H
#define ARDUFLITE_HAL_PLATFORM_IO_H

#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"

namespace arduflite::hal {

enum class PinMode : std::uint8_t { Input, InputPullUp, InputPullDown, Output };

class GpioPin : private NonCopyable
{
public:
    virtual ~GpioPin() = default;

    virtual Status setMode(PinMode mode) = 0;
    [[nodiscard]] virtual bool read() const = 0;
    virtual void write(bool high) = 0;
};

/**
 * @brief PWM output in MICROSECONDS, the hardware's real unit.
 *
 * A degrees-based servo API quantises to ~11 us before the slew
 * limiter ever saw the value.
 */
class PwmOut : private NonCopyable
{
public:
    virtual ~PwmOut() = default;

    virtual Status attach(std::uint16_t minUs, std::uint16_t maxUs,
                          std::uint16_t frameRate_hz = 50) = 0;

    virtual void writeMicroseconds(std::uint16_t us) = 0;

    /// Stop pulsing — FailsafeAction::Release.
    virtual void idle() = 0;

    virtual void detach() = 0;

    [[nodiscard]] virtual std::uint16_t lastMicroseconds() const = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_IO_H
