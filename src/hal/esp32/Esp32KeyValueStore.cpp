/**
 * Esp32KeyValueStore.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32KeyValueStore.h"

#include <Preferences.h>

namespace arduflite::hal::esp32 {

Status Esp32KeyValueStore::begin()
{
    // Open and close once, so a namespace that cannot be created fails here
    // rather than on the first write during config load.
    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/false)) { return Status::IoError; }
    preferences.end();
    return Status::Ok;
}

Status Esp32KeyValueStore::read(const char* key, void* dst, std::size_t capacity,
                                std::size_t& outLen)
{
    outLen = 0;
    if (key == nullptr || dst == nullptr) { return Status::InvalidArg; }

    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/true)) { return Status::NotPresent; }

    const std::size_t stored = preferences.getBytesLength(key);
    if (stored == 0)
    {
        preferences.end();
        return Status::NotPresent;
    }

    if (stored > capacity)
    {
        // Refuse rather than truncate. A short read of a serialised value is
        // not a smaller value, it is a different one.
        preferences.end();
        outLen = stored;
        return Status::NoSpace;
    }

    outLen = preferences.getBytes(key, dst, stored);
    preferences.end();
    return (outLen == stored) ? Status::Ok : Status::IoError;
}

Status Esp32KeyValueStore::write(const char* key, const void* src, std::size_t len)
{
    if (key == nullptr || src == nullptr || len == 0) { return Status::InvalidArg; }

    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/false)) { return Status::IoError; }

    const std::size_t written = preferences.putBytes(key, src, len);
    preferences.end();
    return (written == len) ? Status::Ok : Status::IoError;
}

Status Esp32KeyValueStore::erase(const char* key)
{
    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/false)) { return Status::IoError; }

    const bool removed = preferences.remove(key);
    preferences.end();
    return removed ? Status::Ok : Status::NotPresent;
}

Status Esp32KeyValueStore::eraseAll()
{
    Preferences preferences;
    if (!preferences.begin(_namespace, /*readOnly=*/false)) { return Status::IoError; }

    const bool cleared = preferences.clear();
    preferences.end();
    return cleared ? Status::Ok : Status::IoError;
}

Status Esp32KeyValueStore::commit()
{
    // NVS commits on close, and every operation above closes. Nothing to do —
    // but the method exists because a platform with a write-back cache would
    // need it, and a caller must not have to know which kind it has.
    return Status::Ok;
}

} // namespace arduflite::hal::esp32
