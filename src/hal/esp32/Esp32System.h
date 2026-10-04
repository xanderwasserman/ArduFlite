/**
 * Esp32System.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_HAL_ESP32_SYSTEM_H
#define ARDUFLITE_HAL_ESP32_SYSTEM_H

#include "src/hal/platform/Storage.h"

namespace arduflite::hal::esp32 {

class Esp32System final : public System
{
public:
    [[nodiscard]] ResetCause    resetCause()       const override;
    [[nodiscard]] std::uint32_t freeHeapBytes()    const override;
    [[nodiscard]] std::uint32_t minFreeHeapBytes() const override;
    [[nodiscard]] const char*   uniqueId()         const override;

    [[nodiscard]] const char*   platformName() const override;
    [[nodiscard]] const char*   sdkVersion()   const override;
    [[nodiscard]] std::uint32_t randomWord()         override;

    [[nodiscard]] Status taskReport(char* buffer, std::size_t capacity) const override;

    [[noreturn]] void reboot() override;

private:
    mutable char _id[13]{};
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_SYSTEM_H
