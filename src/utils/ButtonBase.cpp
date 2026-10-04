/**
 * ButtonBase.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ButtonBase.h"

#include "src/utils/Logging.h"

ButtonBase::ButtonBase(arduflite::hal::GpioPin& pin, const arduflite::hal::Clock& clock,
                       bool usePullup, unsigned long debounceMs)
  : _pin(pin)
  , _clock(clock)
  , _usePullup(usePullup)
  , _debounceMs(debounceMs)
{
}

unsigned long ButtonBase::nowMs() const
{
    return static_cast<unsigned long>(_clock.now().time_since_epoch().count() / 1000);
}

void ButtonBase::begin() {
    const arduflite::Status status =
        _pin.setMode(_usePullup ? arduflite::hal::PinMode::InputPullUp
                                : arduflite::hal::PinMode::Input);
    if (status != arduflite::Status::Ok) {
        // The button will read as never pressed. Say so: silently dead controls
        // are indistinguishable from a user who is not pressing anything.
        LOG_ERR("Button: pin setMode failed (%s) - input will not respond",
                arduflite::toString(status));
    }
    reset();
}

void ButtonBase::update() {
    // By default, just do debouncing. Derived classes can override and then call this base method.
    readAndDebounce();
}

void ButtonBase::reset() {
    _rawPressed     = false;
    _stablePressed  = false;
    _lastChangeTime = 0;
}

void ButtonBase::readAndDebounce() {
    unsigned long now = nowMs();

    // 1) Read raw
    // A pulled-up button reads LOW when pressed, so the active level follows
    // the mode. That coupling was implicit in the two digitalRead() branches.
    const bool level = _pin.read();
    const bool currentRaw = _usePullup ? !level : level;

    // 2) If it changed, reset the debounce timer
    if (currentRaw != _rawPressed) {
        _rawPressed = currentRaw;
        _lastChangeTime = now;
    }

    // 3) If stable for >= _debounceMs, adopt the new state
    if ((now - _lastChangeTime) >= _debounceMs) {
        _stablePressed = _rawPressed;
    }
    // else still within debounce period
}
