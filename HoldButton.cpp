// HoldButton.cpp

#include "HoldButton.h"

HoldButton::HoldButton(int pin,
                       unsigned long holdTimeMs,
                       HoldButtonCallback callback,
                       bool usePullup,
                       bool autoReset,
                       unsigned long debounceMs)
    : ButtonBase(pin, usePullup, debounceMs)
    , _holdTime(holdTimeMs)
    , _callback(callback)
    , _autoReset(autoReset)
{
}

void HoldButton::update() {
    // 1) First let the base do its debouncing
    ButtonBase::update(); 
    // or directly: readAndDebounce();

    // 2) Then implement hold-time logic using the stable pressed
    unsigned long now = millis();

    if (isPressed()) {
        if (_pressStart == 0) {
            // Just became pressed
            _pressStart = now;
            _callbackTriggered = false;
        } else {
            // Already pressed
            if (!_callbackTriggered && (now - _pressStart >= _holdTime)) {
                // Crossed the hold threshold
                if (_callback) {
                    _callback();
                }
                _callbackTriggered = true;

                if (_autoReset) {
                    // Re-arm so user can trigger multiple times in one press
                    reset();
                    // If still pressed, treat it as a new press
                    if (isPressed()) {
                        _pressStart = now;
                    }
                }
            }
        }
    } else {
        // Not pressed
        _pressStart = 0;
        _callbackTriggered = false;
    }
}

void HoldButton::reset() {
    ButtonBase::reset();  // calls base class reset to clear debounce states
    _callbackTriggered = false;
    _pressStart = 0;
}
