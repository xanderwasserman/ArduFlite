/**
 * LogRotationPolicy.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/core/LogRotationPolicy.h"

namespace arduflite::drivers {

int LogRotationPolicy::nextIndex(const int* indices, std::size_t count)
{
    int highest = -1;   // sentinel: an empty directory yields index 0
    for (std::size_t i = 0; i < count; ++i)
    {
        if (indices[i] >= 0 && indices[i] <= kMaxIndex && indices[i] > highest)
        {
            highest = indices[i];
        }
    }

    // Common case: room remains above the highest index. No occupancy scan.
    if (highest < kMaxIndex) { return highest + 1; }

    // Rare case: the top index exists, so find the lowest free gap. The bitmap
    // lives only on this path, so the common case never pays for it.
    bool used[kMaxIndex + 1] = {};
    for (std::size_t i = 0; i < count; ++i)
    {
        if (indices[i] >= 0 && indices[i] <= kMaxIndex) { used[indices[i]] = true; }
    }
    for (int i = 0; i <= kMaxIndex; ++i)
    {
        if (!used[i]) { return i; }
    }

    return -1;   // all 1000 taken
}

void LogRotationPolicy::sortAscending(int* indices, std::size_t count)
{
    for (std::size_t i = 1; i < count; ++i)
    {
        const int key = indices[i];
        std::size_t j = i;
        while (j > 0 && indices[j - 1] > key) { indices[j] = indices[j - 1]; --j; }
        indices[j] = key;
    }
}

std::size_t LogRotationPolicy::purgeCount(const int* sortedAscending, std::size_t count,
                                          std::uint32_t freeBytes,
                                          std::uint32_t bytesPerLog) const
{
    (void)sortedAscending;

    if (freeBytes >= minFreeBytes) { return 0; }

    // A store that reports no reclaimable space per log would otherwise loop to
    // the attempt cap deleting everything for no gain.
    if (bytesPerLog == 0) { return 0; }

    std::size_t needed = 0;
    std::uint32_t projected = freeBytes;
    while (projected < minFreeBytes &&
           needed < count &&
           needed < static_cast<std::size_t>(maxPurgeAttempts))
    {
        projected += bytesPerLog;
        ++needed;
    }
    return needed;
}

} // namespace arduflite::drivers
