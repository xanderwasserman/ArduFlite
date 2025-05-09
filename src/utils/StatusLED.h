/**
 * StatusLED.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
// StatusLED.h

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "src/utils/Colors.h"

// Simple RGB status‐LED controller running in its own FreeRTOS task.
// Supports setting a solid color or a blink pattern.
class StatusLED {
public:

    // Create the controller: pin, number of pixels, brightness (0–255)
    StatusLED(uint8_t pin, uint16_t numPixels = 1, uint8_t brightness = 50);
    ~StatusLED();

    // Initialize the strip and launch the LED task (call from setup())
    void begin();

    // Set a solid color (no blinking)
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setColor(const Color &c) { setColor(c.r, c.g, c.b); }

    // Set a repeating blink pattern
    void setPattern(const Pattern& p);

    // Disable LED (turn off)
    void disable();

private:
    static void      taskEntry(void* vp);
    void             run();

    Adafruit_NeoPixel _strip;
    uint8_t           _brightness;
    uint8_t           _r, _g, _b;
    Pattern           _pattern;
    bool              _usePattern;
    TaskHandle_t      _taskHandle;
    SemaphoreHandle_t _mutex;
};

#endif // STATUS_LED_H
