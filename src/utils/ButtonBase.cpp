// ButtonBase.cpp

#include "src/utils/ButtonBase.h"

ButtonBase::ButtonBase(int pin, bool usePullup, unsigned long debounceMs)
  : _pin(pin)
  , _usePullup(usePullup)
  , _debounceMs(debounceMs)
{
}

void ButtonBase::begin() {
    if (_usePullup) {
        pinMode(_pin, INPUT_PULLUP);
    } else {
        pinMode(_pin, INPUT);
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
    unsigned long now = millis();

    // 1) Read raw
    bool currentRaw = _usePullup ? (digitalRead(_pin) == LOW)
                                 : (digitalRead(_pin) == HIGH);

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
