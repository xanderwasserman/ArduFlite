/**
 * Esp32Io.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32Io.h"

#include <Arduino.h>

namespace arduflite::hal::esp32 {

// ── GPIO ────────────────────────────────────────────────────────────────────

Status Esp32GpioPin::setMode(PinMode mode)
{
    if (_pin < 0) { return Status::NotInitialised; }

    switch (mode)
    {
        case PinMode::Input:         pinMode(_pin, INPUT);         break;
        case PinMode::InputPullUp:   pinMode(_pin, INPUT_PULLUP);  break;
        case PinMode::InputPullDown: pinMode(_pin, INPUT_PULLDOWN); break;
        case PinMode::Output:        pinMode(_pin, OUTPUT);        break;
    }
    return Status::Ok;
}

bool Esp32GpioPin::read() const
{
    return (_pin >= 0) && (digitalRead(_pin) == HIGH);
}

void Esp32GpioPin::write(bool high)
{
    if (_pin >= 0) { digitalWrite(_pin, high ? HIGH : LOW); }
}

// ── PWM ─────────────────────────────────────────────────────────────────────

Status Esp32PwmOut::attach(std::uint16_t minUs, std::uint16_t maxUs,
                           std::uint16_t frameRate_hz)
{
    if (_pin < 0)                    { return Status::NotInitialised; }
    if (minUs >= maxUs)              { return Status::InvalidArg; }
    if (frameRate_hz == 0)           { return Status::InvalidArg; }

    _minUs   = minUs;
    _maxUs   = maxUs;
    _frameHz = frameRate_hz;

    if (!ledcAttach(static_cast<std::uint8_t>(_pin), _frameHz, kResolutionBits))
    {
        return Status::IoError;
    }

    _attached = true;
    _lastUs   = 0;
    return Status::Ok;
}

void Esp32PwmOut::writeMicroseconds(std::uint16_t us)
{
    if (!_attached) { return; }

    if (us < _minUs) { us = _minUs; }
    if (us > _maxUs) { us = _maxUs; }

    // Integer duty maths — cheaper than float on a chip with no FPU, and exact.
    // period_us = 1e6 / frameHz;  duty = us * 2^bits / period_us
    const std::uint32_t periodUs = 1000000u / _frameHz;
    const std::uint32_t maxDuty  = (1u << kResolutionBits) - 1u;
    const std::uint32_t duty     = (static_cast<std::uint32_t>(us) * maxDuty) / periodUs;

    ledcWrite(static_cast<std::uint8_t>(_pin), duty);
    _lastUs = us;
}

void Esp32PwmOut::idle()
{
    if (!_attached) { return; }
    ledcWrite(static_cast<std::uint8_t>(_pin), 0);   // no pulse at all
    _lastUs = 0;
}

void Esp32PwmOut::detach()
{
    if (!_attached) { return; }
    ledcDetach(static_cast<std::uint8_t>(_pin));
    _attached = false;
    _lastUs   = 0;
}

} // namespace arduflite::hal::esp32
