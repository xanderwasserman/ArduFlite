/**
 * Esp32SettingsStore.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32SettingsStore.h"

#include <Preferences.h>

#include <cstring>
#include <vector>

#include "src/hal/core/Crc32.h"

namespace arduflite::hal::esp32 {

namespace {

/// Stored layout: the payload followed by a CRC over exactly that payload.
constexpr std::size_t kCrcBytes = sizeof(std::uint32_t);

} // namespace

Status Esp32SettingsStore::save(const char* key, const void* src, std::size_t len)
{
    if (key == nullptr || src == nullptr || len == 0) { return Status::InvalidArg; }

    std::vector<std::uint8_t> buffer(len + kCrcBytes);
    std::memcpy(buffer.data(), src, len);

    const std::uint32_t crc = crc32(src, len);
    std::memcpy(buffer.data() + len, &crc, kCrcBytes);

    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/false)) { return Status::IoError; }

    const std::size_t written = preferences.putBytes(key, buffer.data(), buffer.size());
    preferences.end();

    return (written == buffer.size()) ? Status::Ok : Status::IoError;
}

Status Esp32SettingsStore::load(const char* key, void* dst, std::size_t len)
{
    if (key == nullptr || dst == nullptr || len == 0) { return Status::InvalidArg; }

    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/true)) { return Status::NotPresent; }

    const std::size_t stored = preferences.getBytesLength(key);
    if (stored == 0)
    {
        preferences.end();
        return Status::NotPresent;
    }

    if (stored != len + kCrcBytes)
    {
        // A different size means the blob was written by a different build.
        // Reporting Corrupt rather than reading what fits is deliberate: a
        // truncated read would produce plausible-looking garbage offsets.
        preferences.end();
        return Status::Corrupt;
    }

    std::vector<std::uint8_t> buffer(stored);
    const std::size_t read = preferences.getBytes(key, buffer.data(), buffer.size());
    preferences.end();

    if (read != buffer.size()) { return Status::IoError; }

    std::uint32_t storedCrc = 0;
    std::memcpy(&storedCrc, buffer.data() + len, kCrcBytes);

    if (storedCrc != crc32(buffer.data(), len))
    {
        // The whole reason this class exists. The magic-number scheme it
        // without one, corrupt bytes are handed back as a valid calibration.
        return Status::Corrupt;
    }

    std::memcpy(dst, buffer.data(), len);
    return Status::Ok;
}

Status Esp32SettingsStore::erase(const char* key)
{
    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/false)) { return Status::IoError; }

    const bool removed = preferences.remove(key);
    preferences.end();
    return removed ? Status::Ok : Status::NotPresent;
}

} // namespace arduflite::hal::esp32
