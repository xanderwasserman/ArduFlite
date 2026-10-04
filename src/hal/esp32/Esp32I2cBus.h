/**
 * Esp32I2cBus.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief I2C bus that OWNS its mutex and hands out RegisterDevice handles.
 *
 * This is what makes "the inertial task is the sole I2C owner" structural rather
 * than a rule people have to remember: the bus grants handles at composition
 * time, so a second task cannot acquire one without an edit to Board.cpp.
 */
#ifndef ARDUFLITE_HAL_ESP32_I2CBUS_H
#define ARDUFLITE_HAL_ESP32_I2CBUS_H

#include <array>
#include <cstdint>

#include "src/hal/esp32/Esp32Mutex.h"
#include "src/hal/platform/Buses.h"

namespace arduflite::hal::esp32 {

class Esp32I2cBus;

/// One device address on an Esp32I2cBus.
class Esp32I2cDevice final : public RegisterDevice
{
public:
    Status readRegs (std::uint8_t reg, std::uint8_t* dst, std::size_t len) override;
    Status writeRegs(std::uint8_t reg, const std::uint8_t* src, std::size_t len) override;

    [[nodiscard]] Mutex&      busLock() override;
    [[nodiscard]] const char* busName() const override { return _name; }

private:
    friend class Esp32I2cBus;

    Esp32I2cBus* _bus     = nullptr;
    std::uint8_t _address = 0;
    char         _name[16]{};   ///< "i2c0@0x68"
};

class Esp32I2cBus final : public I2cBus
{
public:
    /// @param sda,scl  GPIO numbers from the board descriptor.
    /// @note constexpr so BoardStorage can be `constinit` — no FreeRTOS or
    ///       Arduino call may happen before begin().
    constexpr Esp32I2cBus(std::int8_t sda, std::int8_t scl) noexcept
        : _sda(sda), _scl(scl) {}

    Status begin(std::uint32_t clockHz) override;

    Result<RegisterDevice*> openDevice(std::uint8_t address7bit) override;
    Status                  probe(std::uint8_t address7bit) override;

    [[nodiscard]] Mutex& lock() noexcept { return _mutex; }

private:
    friend class Esp32I2cDevice;

    Status transfer(std::uint8_t address, std::uint8_t reg,
                    const std::uint8_t* tx, std::size_t txLen,
                    std::uint8_t* rx, std::size_t rxLen);

    std::int8_t  _sda;
    std::int8_t  _scl;
    bool         _started = false;
    Esp32RecursiveMutex _mutex{};   ///< recursive: see RegisterDevice::busLock()

    std::array<Esp32I2cDevice, kMaxDevices> _devices{};
    std::uint8_t                            _deviceCount = 0;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_I2CBUS_H
