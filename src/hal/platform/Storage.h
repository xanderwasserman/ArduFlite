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

    /// Wipe every key in this store's namespace. Needed by "factory reset";
    /// erase(key) alone cannot express it, because the caller does not
    /// necessarily know every key that was ever written — older firmware
    /// versions may have left some behind.
    virtual Status eraseAll() = 0;

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

    /// Chip/board identity and SDK version, for a diagnostics endpoint. Never
    /// parsed — display only.
    [[nodiscard]] virtual const char*   platformName()      const = 0;
    [[nodiscard]] virtual const char*   sdkVersion()        const = 0;

    /**
     * @brief A random 32-bit word from the platform's entropy source.
     *
     * Exists because the web server builds its CSRF token from two of these.
     * That makes it a SECURITY primitive, not a convenience: an implementation
     * backed by rand() or by a boot-time-seeded PRNG would make tokens
     * predictable across reboots, and a device that always boots to the same
     * token is a device with no CSRF protection at all. Implementations must
     * use a hardware entropy source.
     */
    [[nodiscard]] virtual std::uint32_t randomWord() = 0;

    /**
     * @brief Human-readable per-task report, for a CLI diagnostic.
     *
     * Sits alongside resetCause() and freeHeapBytes() because it answers the
     * same kind of question — "what is the platform doing?" — and because the
     * alternative was `vTaskList()` called directly from the CLI, which pinned
     * that command to FreeRTOS.
     *
     * @param buffer   destination, always NUL-terminated on success
     * @param capacity size of @p buffer in bytes
     * @return NotSupported where the platform has no such notion. The caller
     *         reports that; it is not an error worth failing a command over.
     *
     * @note The report's LAYOUT is platform-defined and not parsed anywhere.
     */
    [[nodiscard]] virtual Status taskReport(char* buffer, std::size_t capacity) const = 0;

    [[noreturn]] virtual void reboot() = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_STORAGE_H
