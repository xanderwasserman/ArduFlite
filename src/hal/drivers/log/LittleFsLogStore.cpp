/**
 * LittleFsLogStore.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/log/LittleFsLogStore.h"

#include <cstdio>
#include <cstring>

namespace arduflite::drivers {

void LittleFsLogStore::pathFor(std::uint16_t index, char* out, std::size_t outLen)
{
    snprintf(out, outLen, "/log_%03u.csv", static_cast<unsigned>(index));
}

Status LittleFsLogStore::begin()
{
    if (_mounted) { return Status::Ok; }

    if (!LittleFS.begin())
    {
        // A failed mount on first boot is normal — the partition is unformatted.
        // Formatting is therefore the recovery, not an error, but it destroys
        // any logs that were there, so it is loud.
        if (!LittleFS.format() || !LittleFS.begin())
        {
            return Status::IoError;
        }
    }

    _mounted = true;
    return Status::Ok;
}

std::size_t LittleFsLogStore::listSessions(std::uint16_t* out, std::size_t maxEntries)
{
    if (!_mounted || out == nullptr) { return 0; }

    std::size_t count = 0;
    File root = LittleFS.open("/");
    File entry = root.openNextFile();
    while (entry && count < maxEntries)
    {
        // The filename pointer directly — String would allocate per entry, on a
        // path walked during purge with the file mutex held.
        const char* name = entry.name();
        if (*name == '/') { ++name; }

        unsigned index = 0;
        if (sscanf(name, "log_%03u.csv", &index) == 1 && index <= 999u)
        {
            out[count++] = static_cast<std::uint16_t>(index);
        }
        entry = root.openNextFile();
    }
    root.close();
    return count;
}

Status LittleFsLogStore::openSession(std::uint16_t index)
{
    if (!_mounted) { return Status::NotPresent; }
    if (_file)     { return Status::Busy; }

    char path[kMaxPath];
    pathFor(index, path, sizeof(path));

    _file = LittleFS.open(path, FILE_WRITE);
    return _file ? Status::Ok : Status::IoError;
}

Status LittleFsLogStore::append(const char* data, std::size_t len)
{
    if (!_file)                       { return Status::NotPresent; }
    if (data == nullptr || len == 0)  { return Status::InvalidArg; }

    const std::size_t written = _file.write(reinterpret_cast<const std::uint8_t*>(data), len);

    // A short write means the medium is full. Reporting it is what lets the
    // caller stop logging rather than silently truncating every subsequent row.
    return (written == len) ? Status::Ok : Status::NoSpace;
}

Status LittleFsLogStore::flush()
{
    if (!_file) { return Status::NotPresent; }
    _file.flush();
    return Status::Ok;
}

Status LittleFsLogStore::closeSession()
{
    if (!_file) { return Status::Ok; }
    _file.flush();
    _file.close();
    return Status::Ok;
}

Status LittleFsLogStore::readSession(std::uint16_t index, void* dst, std::size_t maxLen,
                                     std::size_t offset, std::size_t& outLen)
{
    outLen = 0;
    if (!_mounted || dst == nullptr || maxLen == 0) { return Status::InvalidArg; }

    char path[kMaxPath];
    pathFor(index, path, sizeof(path));

    File file = LittleFS.open(path, FILE_READ);
    if (!file) { return Status::NotPresent; }

    // Seeking past the end is how the caller learns it has finished, so it is
    // not an error — it simply reads nothing.
    if (offset > 0 && !file.seek(offset))
    {
        file.close();
        return Status::Ok;
    }

    outLen = file.read(static_cast<std::uint8_t*>(dst), maxLen);
    file.close();
    return Status::Ok;
}

Status LittleFsLogStore::sessionSize(std::uint16_t index, std::uint32_t& bytes)
{
    bytes = 0;
    if (!_mounted) { return Status::NotPresent; }

    char path[kMaxPath];
    pathFor(index, path, sizeof(path));

    File file = LittleFS.open(path, FILE_READ);
    if (!file) { return Status::NotPresent; }

    bytes = static_cast<std::uint32_t>(file.size());
    file.close();
    return Status::Ok;
}

Status LittleFsLogStore::removeSession(std::uint16_t index)
{
    if (!_mounted) { return Status::NotPresent; }

    char path[kMaxPath];
    pathFor(index, path, sizeof(path));
    return LittleFS.remove(path) ? Status::Ok : Status::IoError;
}

Status LittleFsLogStore::usage(std::uint32_t& used, std::uint32_t& total)
{
    if (!_mounted) { used = 0; total = 0; return Status::NotPresent; }

    total = static_cast<std::uint32_t>(LittleFS.totalBytes());
    const std::uint32_t reported = static_cast<std::uint32_t>(LittleFS.usedBytes());

    // Guard a corrupt filesystem reporting used > total, which would make a
    // free-space subtraction wrap to an enormous number and defeat the purge.
    used = (reported <= total) ? reported : total;
    return Status::Ok;
}

Status LittleFsLogStore::formatAll()
{
    (void)closeSession();
    _mounted = false;
    if (!LittleFS.format()) { return Status::IoError; }
    return begin();
}

} // namespace arduflite::drivers
