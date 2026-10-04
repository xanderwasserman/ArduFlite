/**
 * Esp32Uart.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @note No Arduino types in this header — HardwareSerial lives in the .cpp.
 */
#ifndef ARDUFLITE_HAL_ESP32_UART_H
#define ARDUFLITE_HAL_ESP32_UART_H

#include "src/hal/platform/Buses.h"

namespace arduflite::hal::esp32 {

/**
 * @brief UART over HardwareSerial.
 *
 * @note The CRSF receiver and the CRSF telemetry backend SHARE one UART: the
 *       receiver reads, telemetry writes, on the same port with separate pins.
 *       Board owns the instance and hands the same reference to both, which
 *       makes the sharing explicit instead of a comment in ArdufliteApp.cpp.
 */
class Esp32Uart final : public Uart
{
public:
    /// constexpr so BoardStorage can be constinit — no Arduino call before begin().
    constexpr Esp32Uart(std::uint8_t port, std::int8_t rx, std::int8_t tx) noexcept
        : _port(port), _rx(rx), _tx(tx) {}

    Status begin(std::uint32_t baud, bool invertRx = false) override;

    [[nodiscard]] std::size_t available() override;
    [[nodiscard]] std::size_t read(std::uint8_t* dst, std::size_t maxLen) override;
    std::size_t               write(const std::uint8_t* src, std::size_t len) override;
    void                      flush() override;

private:
    std::uint8_t _port;
    std::int8_t  _rx;
    std::int8_t  _tx;
    bool         _started = false;
    void*        _serial  = nullptr;   ///< HardwareSerial*, type-erased
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_UART_H
