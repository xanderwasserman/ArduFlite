/**
 * Esp32Uart.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32Uart.h"

#include <Arduino.h>
#include <HardwareSerial.h>

namespace arduflite::hal::esp32 {

namespace {

/// The core's instance for a hardware UART. UART0 is available only when the
/// console runs over USB; otherwise it IS the console. Which ports a board may
/// use is the descriptor's business, checked by BoardValidate.
HardwareSerial* portInstance(std::uint8_t port)
{
    switch (port)
    {
#if ARDUINO_USB_CDC_ON_BOOT
        case 0:  return &Serial0;
#endif
#if SOC_UART_NUM > 1
        case 1:  return &Serial1;
#endif
#if SOC_UART_NUM > 2
        case 2:  return &Serial2;
#endif
        default: return nullptr;
    }
}

inline HardwareSerial* asSerial(void* p) { return static_cast<HardwareSerial*>(p); }

} // namespace

Status Esp32Uart::begin(std::uint32_t baud, bool invertRx)
{
    if (_started) { return Status::Ok; }

    _serial = portInstance(_port);
    if (_serial == nullptr) { return Status::InvalidArg; }

    asSerial(_serial)->begin(baud, SERIAL_8N1, _rx, _tx, invertRx);
    _started = true;
    return Status::Ok;
}

std::size_t Esp32Uart::available()
{
    if (!_started) { return 0; }
    const int n = asSerial(_serial)->available();
    return (n > 0) ? static_cast<std::size_t>(n) : 0;
}

std::size_t Esp32Uart::read(std::uint8_t* dst, std::size_t maxLen)
{
    if (!_started || dst == nullptr || maxLen == 0) { return 0; }
    const std::size_t ready = available();
    if (ready == 0) { return 0; }
    return asSerial(_serial)->read(dst, (ready < maxLen) ? ready : maxLen);
}

std::size_t Esp32Uart::writable()
{
    if (!_started) { return 0; }
    const int n = asSerial(_serial)->availableForWrite();
    return (n > 0) ? static_cast<std::size_t>(n) : 0;
}

std::size_t Esp32Uart::write(const std::uint8_t* src, std::size_t len)
{
    if (!_started || src == nullptr || len == 0) { return 0; }
    return asSerial(_serial)->write(src, len);
}

void Esp32Uart::flush()
{
    if (_started) { asSerial(_serial)->flush(); }
}

} // namespace arduflite::hal::esp32
