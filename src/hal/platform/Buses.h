/**
 * Buses.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief I2C, SPI, CAN and UART. No Arduino types: no Print, no Stream, no String.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_BUSES_H
#define ARDUFLITE_HAL_PLATFORM_BUSES_H

#include <cstddef>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Result.h"
#include "src/hal/platform/ByteStream.h"
#include "src/hal/platform/Mutex.h"
#include "src/hal/platform/RegisterDevice.h"

namespace arduflite::hal {

class I2cBus : private NonCopyable
{
public:
    /// Device handles live in a fixed array inside the bus — no heap after boot.
    static constexpr std::uint8_t kMaxDevices = 8;

    virtual ~I2cBus() = default;

    virtual Status begin(std::uint32_t clockHz) = 0;

    /// Handles are carved out at composition time and live as long as the bus.
    /// Returns Status::NoSpace once kMaxDevices are open.
    virtual Result<RegisterDevice*> openDevice(std::uint8_t address7bit) = 0;

    /// ACK test only — does not disturb device state.
    virtual Status probe(std::uint8_t address7bit) = 0;
};

class SpiBus : private NonCopyable
{
public:
    static constexpr std::uint8_t kMaxDevices = 4;

    virtual ~SpiBus() = default;

    virtual Status begin() = 0;
    virtual Result<RegisterDevice*> openDevice(std::uint8_t  csPinIndex,
                                               std::uint32_t clockHz,
                                               std::uint8_t  mode) = 0;
};

struct CanFrame
{
    std::uint32_t id         = 0;
    std::uint8_t  data[8]    = {};
    std::uint8_t  length     = 0;
    bool          extendedId = false;
};

/**
 * @brief Declared so the actuator abstraction is demonstrably transport-neutral.
 *        Not implemented until something needs it; the ESP32-C3's TWAI controller
 *        makes an Esp32CanBus a real option rather than a hypothetical one.
 */
class CanBus : private NonCopyable
{
public:
    virtual ~CanBus() = default;

    virtual Status begin(std::uint32_t bitrate_bps) = 0;
    virtual Status send(const CanFrame& frame) = 0;

    /// True if a frame was waiting.
    [[nodiscard]] virtual bool receive(CanFrame& out) = 0;

    [[nodiscard]] virtual Mutex& busLock() = 0;
};

class Uart : public ByteStream
{
public:
    virtual Status begin(std::uint32_t baud, bool invertRx = false) = 0;

    /// Block until every queued byte has left the wire.
    virtual void flush() = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_BUSES_H
