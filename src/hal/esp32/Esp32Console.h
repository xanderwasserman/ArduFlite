/**
 * Esp32Console.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief device::Console over the Arduino USB serial port.
 */
#ifndef ARDUFLITE_HAL_ESP32_CONSOLE_H
#define ARDUFLITE_HAL_ESP32_CONSOLE_H

#include "src/hal/device/Peripherals.h"

namespace arduflite::hal::esp32 {

class Esp32Console final : public device::Console
{
public:
    /// constexpr: this lives in BoardStorage, which is constinit. Serial is a
    /// global the core already constructs, so there is nothing to own here.
    constexpr Esp32Console() noexcept = default;

    Status begin(std::uint32_t baud);

    std::size_t write(const char* s, std::size_t len) override;
    [[nodiscard]] std::size_t available() const override;
    [[nodiscard]] int readByte() override;
    void flushOutput() override;

private:
    bool _started = false;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_CONSOLE_H
