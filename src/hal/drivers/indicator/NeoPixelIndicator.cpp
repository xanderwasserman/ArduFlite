/**
 * NeoPixelIndicator.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/indicator/NeoPixelIndicator.h"

#include <mutex>

#include "src/hal/board/Board.h"

namespace arduflite::drivers {

namespace {

void taskEntry(void* argument)
{
    static_cast<NeoPixelIndicator*>(argument)->runTaskLoop();
}

} // namespace

Status NeoPixelIndicator::begin()
{
    if (_task != nullptr) { return Status::Ok; }

    Result<hal::Mutex*> mutex = board::Board::instance().allocMutex();
    if (!mutex) { return mutex.status(); }
    _mutex = mutex.value();

    _strip.begin();
    _strip.setBrightness(_brightness);
    _strip.clear();
    _strip.show();

    hal::TaskConfig config;
    config.name       = "LED Task";
    config.stackBytes = 2048;
    config.priority   = hal::Priority::Indicator;

    Result<hal::Task*> task = _scheduler.spawn(config, &taskEntry, this);
    if (!task) { return task.status(); }

    _task = task.value();
    return Status::Ok;
}

void NeoPixelIndicator::setColour(device::Rgb c)
{
    if (_mutex == nullptr) { return; }
    std::lock_guard lock(*_mutex);
    _colour     = c;
    _usePattern = false;
}

void NeoPixelIndicator::setPattern(const device::BlinkPattern& p)
{
    if (_mutex == nullptr) { return; }
    std::lock_guard lock(*_mutex);
    _pattern = p;

    // A pattern with no on- or off-time is a solid colour, not a zero-delay
    // blink loop — which would spin the task at full rate against the pixel.
    _usePattern = (p.onMs + p.offMs) > 0;
    _colour     = p.colour;
}

void NeoPixelIndicator::off()
{
    setColour(device::Rgb{ 0, 0, 0 });
}

void NeoPixelIndicator::runTaskLoop()
{
    using namespace std::chrono_literals;

    for (;;)
    {
        device::BlinkPattern pattern{};
        device::Rgb          colour{};
        bool                 usePattern = false;

        // Snapshot under the lock, then act outside it: the blink delays are
        // hundreds of milliseconds, and holding the mutex across them would
        // block every setPattern() caller for that long.
        {
            std::lock_guard lock(*_mutex);
            pattern    = _pattern;
            colour     = _colour;
            usePattern = _usePattern;
        }

        if (usePattern)
        {
            _strip.setPixelColor(0, _strip.Color(pattern.colour.r, pattern.colour.g, pattern.colour.b));
            _strip.show();
            _scheduler.sleepFor(std::chrono::milliseconds{ pattern.onMs });

            _strip.clear();
            _strip.show();
            _scheduler.sleepFor(std::chrono::milliseconds{ pattern.offMs });
        }
        else
        {
            _strip.setPixelColor(0, _strip.Color(colour.r, colour.g, colour.b));
            _strip.show();
            _scheduler.sleepFor(50ms);
        }
    }
}

} // namespace arduflite::drivers
