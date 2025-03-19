// HoldButton.h

#ifndef HOLD_BUTTON_H
#define HOLD_BUTTON_H

#include "src/utils/ButtonBase.h"

typedef void (*HoldButtonCallback)();

class HoldButton : public ButtonBase {
public:
    /**
     * @param pin          Digital pin
     * @param holdTimeMs   How long the button must be pressed to trigger callback
     * @param callback     Function to call when hold time is exceeded
     * @param usePullup    If true, configure pin as INPUT_PULLUP
     * @param autoReset    If true, after callback triggers, reset so it can trigger again
     * @param debounceMs   Software debounce period in ms
     */
    HoldButton(int pin,
               unsigned long holdTimeMs,
               HoldButtonCallback callback,
               bool usePullup    = true,
               bool autoReset    = false,
               unsigned long debounceMs = 30);

    /**
     * Override the base update() to add hold-time logic.
     */
    void update() override;

    /**
     * Reset state, plus base class reset
     */
    void reset() override;

private:
    unsigned long _holdTime;
    HoldButtonCallback _callback;
    bool _autoReset;

    bool _callbackTriggered = false;
    unsigned long _pressStart = 0;
};

#endif // HOLD_BUTTON_H
