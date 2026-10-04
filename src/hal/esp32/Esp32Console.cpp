/**
 * Esp32Console.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32Console.h"

#include <Arduino.h>

namespace arduflite::hal::esp32 {

Status Esp32Console::begin(std::uint32_t baud)
{
    if (_started) { return Status::Ok; }
    Serial.begin(baud);
    _started = true;
    return Status::Ok;
}

std::size_t Esp32Console::available()
{
    const int n = Serial.available();
    return (n > 0) ? static_cast<std::size_t>(n) : 0;
}

std::size_t Esp32Console::read(std::uint8_t* dst, std::size_t maxLen)
{
    if (dst == nullptr || maxLen == 0) { return 0; }
    const std::size_t ready = available();
    if (ready == 0) { return 0; }
    return Serial.read(dst, (ready < maxLen) ? ready : maxLen);
}

std::size_t Esp32Console::writable()
{
    const int n = Serial.availableForWrite();
    return (n > 0) ? static_cast<std::size_t>(n) : 0;
}

std::size_t Esp32Console::write(const std::uint8_t* src, std::size_t len)
{
    if (src == nullptr || len == 0) { return 0; }
    return Serial.write(src, len);
}

void Esp32Console::flushOutput()
{
    Serial.flush();
}

} // namespace arduflite::hal::esp32

