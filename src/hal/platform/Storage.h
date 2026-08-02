/**
 * Storage.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Key-value store (NVS), file store (LittleFS) and system info.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_STORAGE_H
#define ARDUFLITE_HAL_PLATFORM_STORAGE_H

#include <cstddef>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Result.h"

namespace arduflite::hal {

class KeyValueStore : private NonCopyable
{
public:
    virtual ~KeyValueStore() = default;

    virtual Status begin() = 0;
    virtual Status read (const char* key, void* dst, std::size_t capacity, std::size_t& outLen) = 0;
    virtual Status write(const char* key, const void* src, std::size_t len) = 0;
    virtual Status erase(const char* key) = 0;
    virtual Status commit() = 0;
};

struct FileInfo
{
    char          name[32] = {};
    std::uint32_t sizeBytes = 0;
};

class FileStore : private NonCopyable
{
public:
    virtual ~FileStore() = default;

    virtual Status begin() = 0;

    virtual Result<int>         open  (const char* path, bool forWrite) = 0;
    virtual Status              append(int handle, const void* src, std::size_t len) = 0;
    virtual Result<std::size_t> read  (int handle, void* dst, std::size_t maxLen) = 0;
    virtual Status              close (int handle) = 0;
    virtual Status              remove(const char* path) = 0;

    virtual std::size_t list(FileInfo* out, std::size_t maxEntries) = 0;
    virtual Status      format() = 0;
    virtual Status      usage(std::uint32_t& usedBytes, std::uint32_t& totalBytes) = 0;
};

enum class ResetCause : std::uint8_t { PowerOn, Software, Panic, Watchdog, Brownout, Unknown };

constexpr const char* toString(ResetCause c) noexcept
{
    switch (c)
    {
        case ResetCause::PowerOn:  return "PowerOn";
        case ResetCause::Software: return "Software";
        case ResetCause::Panic:    return "Panic";
        case ResetCause::Watchdog: return "Watchdog";
        case ResetCause::Brownout: return "Brownout";
        case ResetCause::Unknown:  return "Unknown";
    }
    return "Unknown";
}

class System : private NonCopyable
{
public:
    virtual ~System() = default;

    [[nodiscard]] virtual ResetCause    resetCause()        const = 0;
    [[nodiscard]] virtual std::uint32_t freeHeapBytes()     const = 0;
    [[nodiscard]] virtual std::uint32_t minFreeHeapBytes()  const = 0;
    [[nodiscard]] virtual const char*   uniqueId()          const = 0;

    [[noreturn]] virtual void reboot() = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_STORAGE_H
