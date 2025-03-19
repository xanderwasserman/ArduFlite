// ButtonBase.h

#ifndef BUTTON_BASE_H
#define BUTTON_BASE_H

#include <Arduino.h>

class ButtonBase {
public:
    /**
     * Constructor
     * @param pin        Digital pin for this button
     * @param usePullup  If true, configure as INPUT_PULLUP
     * @param debounceMs Debounce time in milliseconds
     */
    ButtonBase(int pin, bool usePullup = true, unsigned long debounceMs = 30);

    /**
     * Call in setup() to initialize pin mode.
     */
    virtual void begin();

    /**
     * A virtual update() that derived classes may override or extend.
     * The base method implements debouncing.
     */
    virtual void update();

    /**
     * Resets internal states. Derived classes can override if needed.
     */
    virtual void reset();

    /**
     * Returns the debounced pressed state: true if pressed, false if not pressed.
     */
    bool isPressed() const { return _stablePressed; }

protected:
    /**
     * The raw read function + logic that sets stable pressed with debounce.
     * Called inside update(). Derived classes can also call it directly if needed.
     */
    void readAndDebounce();

    int  _pin;
    bool _usePullup;
    unsigned long _debounceMs;

    // Debounce logic
    bool _rawPressed      = false;  // immediate raw reading
    bool _stablePressed   = false;  // debounced state
    unsigned long _lastChangeTime   = 0; // last time raw state changed
};

#endif // BUTTON_BASE_H
