/**
 * ButtonBase.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef BUTTON_BASE_H
#define BUTTON_BASE_H

#include "src/hal/platform/Clock.h"
#include "src/hal/platform/Io.h"


class ButtonBase {
public:
    /**
     * @param pin        the GPIO, owned by Board. Must outlive this button; it
     *                   points into BoardStorage, a file-scope object.
     * @param clock      time source for debounce and hold timing. Injected for
     *                   the same reason as the pin: buttons are constructed at
     *                   file scope, before Board::begin() has run.
     * @param usePullup  selects the pin mode AND the active level: with a
     *                   pull-up the button reads LOW when pressed.
     * @param debounceMs debounce time, in milliseconds.
     */
    ButtonBase(arduflite::hal::GpioPin& pin, const arduflite::hal::Clock& clock,
               bool usePullup = true, unsigned long debounceMs = 30);

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

    /// Milliseconds since boot, from the injected clock.
    ///
    /// The clock is INJECTED rather than read from the board singleton, for the
    /// same reason the pin is: these objects are constructed at file scope,
    /// before Board::begin() has run. Holding a reference is safe there;
    /// calling into the singleton on every update() would not be testable and
    /// would hide the dependency.
    [[nodiscard]] unsigned long nowMs() const;

    arduflite::hal::GpioPin& _pin;
    const arduflite::hal::Clock& _clock;
    bool _usePullup;
    unsigned long _debounceMs;

    // Debounce logic
    bool _rawPressed      = false;  // immediate raw reading
    bool _stablePressed   = false;  // debounced state
    unsigned long _lastChangeTime   = 0; // last time raw state changed
};

#endif // BUTTON_BASE_H
