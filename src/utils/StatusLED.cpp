/**
 * StatusLED.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#include "StatusLED.h"

StatusLED::StatusLED(uint8_t pin, uint16_t numPixels, uint8_t brightness)
    : _strip(numPixels, pin, NEO_RGB + NEO_KHZ800),
      _brightness(brightness),
      _r(0), _g(0), _b(0),
      _pattern{0,0,0,0,0},
      _usePattern(false),
      _taskHandle(nullptr)
{
    // only create mutex here
    _mutex = xSemaphoreCreateMutex();
}

StatusLED::~StatusLED() 
{
    if (_taskHandle) {
        vTaskDelete(_taskHandle);
    }
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }
}

void StatusLED::begin() 
{
    // now that Arduino is up, initialise the strip:
    _strip.begin();
    _strip.setBrightness(_brightness);
    _strip.show();

    // launch the FreeRTOS task
    xTaskCreate(
        taskEntry,
        "StatusLED",
        4096,
        this,
        tskIDLE_PRIORITY + 1,
        &_taskHandle
    );
}

void StatusLED::setColor(uint8_t r, uint8_t g, uint8_t b) 
{
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        _usePattern = false;
        _r = r; _g = g; _b = b;
        xSemaphoreGive(_mutex);
    }
}

void StatusLED::setPattern(const Pattern& p) 
{
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        _pattern = p;
        _usePattern = true;
        xSemaphoreGive(_mutex);
    }
}

void StatusLED::disable() 
{
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        _usePattern = false;
        _r = _g = _b = 0;
        xSemaphoreGive(_mutex);
    }
}

void StatusLED::taskEntry(void* vp) 
{
    static_cast<StatusLED*>(vp)->run();
}

void StatusLED::run() 
{
    while (true) {
        bool        localUsePattern;
        Pattern     localPattern;
        uint8_t     localR, localG, localB;

        // snapshot shared state under lock
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            localUsePattern = _usePattern;
            localPattern    = _pattern;
            localR = _r; localG = _g; localB = _b;
            xSemaphoreGive(_mutex);
        }

        if (localUsePattern && (localPattern.on_ms + localPattern.off_ms > 0)) {
            // blink ON
            _strip.setPixelColor(0, _strip.Color(localPattern.r, localPattern.g, localPattern.b));
            _strip.show();
            vTaskDelay(pdMS_TO_TICKS(localPattern.on_ms));
            // blink OFF
            _strip.clear();
            _strip.show();
            vTaskDelay(pdMS_TO_TICKS(localPattern.off_ms));
        } 
        else {
            // solid
            _strip.setPixelColor(0, _strip.Color(localR, localG, localB));
            _strip.show();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
