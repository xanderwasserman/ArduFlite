/**
 * LittleFsLogStore.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief device::LogStore over LittleFS.
 *
 * Files are named `/log_NNN.csv`. Deliberately the same layout the flash
 * telemetry used directly before this extraction, so logs already on a device
 * stay listable and readable.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_LOG_LITTLEFS_LOG_STORE_H
#define ARDUFLITE_HAL_DRIVERS_LOG_LITTLEFS_LOG_STORE_H

#include <LittleFS.h>

#include "src/hal/device/Peripherals.h"

namespace arduflite::drivers {

class LittleFsLogStore final : public device::LogStore
{
public:
    /// Longest `/log_NNN.csv` plus terminator.
    static constexpr std::size_t kMaxPath = 32;

    Status begin() override;
    std::size_t listSessions(std::uint16_t* out, std::size_t maxEntries) override;
    Status openSession(std::uint16_t index) override;
    Status append(const char* data, std::size_t len) override;
    Status flush() override;
    Status closeSession() override;
    [[nodiscard]] bool isOpen() const override { return static_cast<bool>(_file); }
    Status readSession(std::uint16_t index, void* dst, std::size_t maxLen,
                       std::size_t offset, std::size_t& outLen) override;
    Status sessionSize(std::uint16_t index, std::uint32_t& bytes) override;
    Status removeSession(std::uint16_t index) override;
    Status usage(std::uint32_t& used, std::uint32_t& total) override;
    Status formatAll() override;

    /// The path for an index, exposed so callers can log which file they mean.
    static void pathFor(std::uint16_t index, char* out, std::size_t outLen);

private:
    File _file;
    bool _mounted = false;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_LOG_LITTLEFS_LOG_STORE_H
