/**
 * MultiTapButton.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/MultiTapButton.h"

MultiTapButton::MultiTapButton(int pin,
                               unsigned long maxTapIntervalMs,
                               int requiredTaps,
                               MultiTapCallback callback,
                               bool usePullup,
                               unsigned long debounceMs)
    : ButtonBase(pin, usePullup, debounceMs)
    , _maxTapInterval(maxTapIntervalMs)
    , _requiredTaps(requiredTaps)
    , _callback(callback)
{
}

void MultiTapButton::update() {
    // 1) Debounce using the base class
    ButtonBase::update();

    unsigned long now = millis();
    bool currentlyPressed = isPressed();

    // 2) Detect a transition from not pressed -> pressed
    if (!_wasPressed && currentlyPressed) {
        _tapCount++;
        _lastTapTime = now;
    }

    // 3) If we've started tapping, but no new tap has arrived in the interval
    if (_tapCount > 0 && (now - _lastTapTime) > _maxTapInterval) {
        finalizeTapCount();
    }

    _wasPressed = currentlyPressed;
}

void MultiTapButton::finalizeTapCount() {
    // If user exactly met the required number of taps, call the callback
    if (_tapCount == _requiredTaps) {
        if (_callback) {
            _callback();
        }
    }
    reset(); // reset for the next sequence
}

void MultiTapButton::reset() {
    _tapCount    = 0;
    _lastTapTime = 0;
    _wasPressed  = false;

    // Also reset base class debouncing
    ButtonBase::reset();
}
