/**
 * RegisterDevice.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Bus-agnostic register access. A driver written against this works over
 *        I2C or SPI unchanged — the one part of AP_HAL::Device worth copying.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_REGISTERDEVICE_H
#define ARDUFLITE_HAL_PLATFORM_REGISTERDEVICE_H

#include <cstddef>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"
#include "src/hal/platform/Mutex.h"

namespace arduflite::hal {

class RegisterDevice : private NonCopyable
{
public:
    virtual ~RegisterDevice() = default;

    virtual Status readRegs (std::uint8_t reg, std::uint8_t* dst, std::size_t len) = 0;
    virtual Status writeRegs(std::uint8_t reg, const std::uint8_t* src, std::size_t len) = 0;

    Status readReg (std::uint8_t reg, std::uint8_t& out) { return readRegs(reg, &out, 1); }
    Status writeReg(std::uint8_t reg, std::uint8_t val)  { return writeRegs(reg, &val, 1); }

    /// Bus-wide mutex. A driver needing several atomic transactions takes
    /// std::unique_lock(dev.busLock()); single transactions lock internally.
    [[nodiscard]] virtual Mutex& busLock() = 0;

    /// e.g. "i2c0@0x68" — for logs and the boot inventory.
    [[nodiscard]] virtual const char* busName() const = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_REGISTERDEVICE_H
