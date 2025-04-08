/**
 * MultiTapButton.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef MULTI_TAP_BUTTON_H
#define MULTI_TAP_BUTTON_H

#include "src/utils/ButtonBase.h"

typedef void (*MultiTapCallback)();

/**
 * A "single-target" multi-tap detector:
 *  - Each instance monitors one pin and looks for a specific tap count.
 *  - If the user presses the button the required number of times
 *    (within a configurable interval), it triggers a callback.
 *  - After that interval, if the required tap count wasn't reached,
 *    the count resets.
 */
class MultiTapButton : public ButtonBase {
public:
    /**
     * Constructor
     * 
     * @param pin                The digital pin for this button
     * @param maxTapIntervalMs   If no new tap is detected within this time window,
     *                           finalize the current sequence
     * @param requiredTaps       The exact number of taps required to trigger the callback
     * @param callback           Function to call when the required tap count is reached
     * @param usePullup          If true, configure pin as INPUT_PULLUP
     * @param debounceMs         Debounce period (ms) for the base ButtonBase class
     */
    MultiTapButton(int pin,
                   unsigned long maxTapIntervalMs,
                   int requiredTaps,
                   MultiTapCallback callback,
                   bool usePullup = true,
                   unsigned long debounceMs = 30);

    /**
     * Update logic: calls base debouncing, detects new presses,
     * and checks if we should finalize taps after a timeout.
     */
    void update() override;

    /**
     * Resets internal tap-count state (and base class states).
     */
    void reset() override;

private:
    unsigned long _maxTapInterval;
    int           _requiredTaps;
    MultiTapCallback _callback;

    // Internal tracking for counting taps
    int           _tapCount     = 0;
    unsigned long _lastTapTime  = 0;
    bool          _wasPressed   = false;

    // Called when no new tap arrives before maxTapInterval
    void finalizeTapCount();
};

#endif // MULTI_TAP_BUTTON_H
