/**
 * Esp32I2cBus.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32I2cBus.h"

#include <Arduino.h>
#include <Wire.h>

#include <cstdio>
#include <mutex>

namespace arduflite::hal::esp32 {

// ── Esp32I2cDevice ──────────────────────────────────────────────────────────

Status Esp32I2cDevice::readRegs(std::uint8_t reg, std::uint8_t* dst, std::size_t len)
{
    if (_bus == nullptr)            { return Status::NotInitialised; }
    if (dst == nullptr || len == 0) { return Status::InvalidArg; }
    return _bus->transfer(_address, reg, nullptr, 0, dst, len);
}

Status Esp32I2cDevice::writeRegs(std::uint8_t reg, const std::uint8_t* src, std::size_t len)
{
    if (_bus == nullptr)            { return Status::NotInitialised; }
    if (src == nullptr || len == 0) { return Status::InvalidArg; }
    return _bus->transfer(_address, reg, src, len, nullptr, 0);
}

Mutex& Esp32I2cDevice::busLock()
{
    return _bus->lock();
}

// ── Esp32I2cBus ─────────────────────────────────────────────────────────────

Status Esp32I2cBus::begin(std::uint32_t clockHz)
{
    if (_started) { return Status::Ok; }

    ARDUFLITE_TRY(_mutex.begin());

    if (!Wire.begin(_sda, _scl)) { return Status::IoError; }
    Wire.setClock(clockHz);
    _started = true;
    return Status::Ok;
}

Result<RegisterDevice*> Esp32I2cBus::openDevice(std::uint8_t address7bit)
{
    if (address7bit > 0x7F)          { return Status::InvalidArg; }
    if (_deviceCount >= kMaxDevices) { return Status::NoSpace; }

    Esp32I2cDevice& dev = _devices[_deviceCount];
    dev._bus     = this;
    dev._address = address7bit;
    std::snprintf(dev._name, sizeof(dev._name), "i2c@0x%02X", address7bit);

    ++_deviceCount;
    return static_cast<RegisterDevice*>(&dev);
}

Status Esp32I2cBus::probe(std::uint8_t address7bit)
{
    if (!_started) { return Status::NotInitialised; }

    std::unique_lock guard(_mutex, std::chrono::milliseconds{ 5 });
    if (!guard) { return Status::Busy; }

    Wire.beginTransmission(address7bit);
    return (Wire.endTransmission() == 0) ? Status::Ok : Status::NotPresent;
}

Status Esp32I2cBus::transfer(std::uint8_t address, std::uint8_t reg,
                             const std::uint8_t* tx, std::size_t txLen,
                             std::uint8_t* rx, std::size_t rxLen)
{
    if (!_started) { return Status::NotInitialised; }

    // Single transactions lock internally. The mutex is RECURSIVE, so a driver
    // grouping several transactions under std::unique_lock(dev.busLock()) nests
    // here harmlessly instead of self-deadlocking.
    std::unique_lock guard(_mutex, std::chrono::milliseconds{ 5 });
    if (!guard) { return Status::Busy; }

    Wire.beginTransmission(address);
    if (Wire.write(reg) != 1) { return Status::IoError; }

    if (tx != nullptr && txLen > 0)
    {
        if (Wire.write(tx, txLen) != txLen) { return Status::IoError; }
    }

    // Repeated START when a read follows, STOP otherwise.
    const bool sendStop = (rx == nullptr || rxLen == 0);
    if (Wire.endTransmission(sendStop) != 0) { return Status::IoError; }

    if (sendStop) { return Status::Ok; }

    const std::size_t got = Wire.requestFrom(static_cast<int>(address),
                                             static_cast<int>(rxLen));
    if (got != rxLen) { return Status::IoError; }

    for (std::size_t i = 0; i < rxLen; ++i)
    {
        const int b = Wire.read();
        if (b < 0) { return Status::IoError; }
        rx[i] = static_cast<std::uint8_t>(b);
    }
    return Status::Ok;
}

} // namespace arduflite::hal::esp32
