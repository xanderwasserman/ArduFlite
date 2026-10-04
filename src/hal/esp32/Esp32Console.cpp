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

std::size_t Esp32Console::write(const char* s, std::size_t len)
{
    if (s == nullptr || len == 0) { return 0; }
    return Serial.write(reinterpret_cast<const std::uint8_t*>(s), len);
}

std::size_t Esp32Console::available() const
{
    // Serial::available() is not const, and the port is a global anyway.
    return static_cast<std::size_t>(Serial.available());
}

int Esp32Console::readByte()
{
    return Serial.read();   // already -1 when empty
}

void Esp32Console::flushOutput()
{
    Serial.flush();
}

} // namespace arduflite::hal::esp32

