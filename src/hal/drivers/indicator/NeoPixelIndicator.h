/**
 * NeoPixelIndicator.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief device::Indicator over a WS2812-family pixel.
 *
 * A task that either holds a solid colour or alternates on/off. Sitting behind
 * device::Indicator is what keeps Adafruit_NeoPixel out of StateManagement and
 * the button callbacks.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_NEOPIXEL_INDICATOR_H
#define ARDUFLITE_HAL_DRIVERS_NEOPIXEL_INDICATOR_H

#include <Adafruit_NeoPixel.h>

#include "src/hal/device/Peripherals.h"
#include "src/hal/platform/Mutex.h"
#include "src/hal/platform/Scheduler.h"

namespace arduflite::drivers {

class NeoPixelIndicator final : public device::Indicator
{
public:
    /// @param scheduler used once, by begin(), to spawn the blink task.
    NeoPixelIndicator(std::uint8_t pin, hal::Scheduler& scheduler,
                      std::uint16_t pixelCount = 1, std::uint8_t brightness = 50)
        : _strip(pixelCount, pin, NEO_GRB + NEO_KHZ800)
        , _scheduler(scheduler)
        , _brightness(brightness) {}

    Status begin() override;
    void   setColour(device::Rgb c) override;
    void   setPattern(const device::BlinkPattern& p) override;
    void   off() override;

    /// Task body. Public only so the entry trampoline can reach it.
    void runTaskLoop();

private:
    Adafruit_NeoPixel _strip;
    hal::Scheduler&   _scheduler;
    std::uint8_t      _brightness;

    /// Guards the fields below, written by callers and read by the task.
    hal::Mutex*  _mutex = nullptr;
    hal::Task*   _task  = nullptr;

    device::BlinkPattern _pattern{};
    device::Rgb          _colour{};
    bool                 _usePattern = false;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_NEOPIXEL_INDICATOR_H
