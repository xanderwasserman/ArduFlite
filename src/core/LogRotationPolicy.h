/**
 * LogRotationPolicy.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Which log index to use next, and what to delete to make room.
 *
 * Split out from the filesystem deliberately. These two rules — monotonic index
 * allocation and purge-oldest-when-low — are the parts of flash logging that
 * are almost impossible to exercise on hardware: reaching the purge path means
 * filling a 1.9 MB partition, and reaching index exhaustion means creating 1000
 * files. They are trivial to drive in memory.
 *
 * Pure functions over an index list. No filesystem, no I/O, no allocation.
 *
 * @note Lives in src/core/, NOT src/hal/drivers/. It was written there first and
 *       the layering check rejected it immediately — correctly. This is policy,
 *       not a driver: it touches no hardware, and both the flight-layer flash
 *       telemetry and any future LogStore driver need it. Same reasoning as
 *       CrsfProtocol.h (ADR-028) — the rule is "flight code must not depend on
 *       a specific device driver", and a rule about log numbering is not a
 *       device.
 */
#ifndef ARDUFLITE_CORE_LOG_ROTATION_POLICY_H
#define ARDUFLITE_CORE_LOG_ROTATION_POLICY_H

#include <cstddef>
#include <cstdint>

namespace arduflite::drivers {

struct LogRotationPolicy
{
    /// Highest permissible log index. log_000.csv .. log_999.csv.
    static constexpr int kMaxIndex = 999;

    /// Below this much free space, purge oldest logs before starting a new one.
    /// At 10 Hz and ~175 bytes per row, 300 KB is roughly three minutes of
    /// headroom — enough that a flight starting near the limit still records.
    std::uint32_t minFreeBytes = 300u * 1024u;

    /// Cap on how many logs one purge pass will delete, as a guard against a
    /// corrupt filesystem reporting free space that never rises.
    int maxPurgeAttempts = 20;

    /**
     * @brief The index a new session should use.
     *
     * Indices grow monotonically as (highest + 1) until the space is full; only
     * then is the lowest free gap reused. Deleting a mid-range log does NOT
     * reclaim its index while room remains above — that keeps log ordering
     * intuitive, so log_007 is always older than log_008.
     *
     * @param indices  the indices currently in use, in any order
     * @return the index to use, or -1 if all 1000 are taken
     */
    [[nodiscard]] static int nextIndex(const int* indices, std::size_t count);

    /**
     * @brief How many of the oldest logs to delete to reach minFreeBytes.
     *
     * @param sortedAscending  indices in use, ASCENDING (oldest first)
     * @param freeBytes        current free space
     * @param bytesPerLog      caller's estimate of what each delete reclaims;
     *                         a real store re-measures instead and stops early
     * @return how many entries from the front of the list to remove
     */
    [[nodiscard]] std::size_t purgeCount(const int* sortedAscending, std::size_t count,
                                         std::uint32_t freeBytes,
                                         std::uint32_t bytesPerLog) const;

    /// Ascending insertion sort, in place. Small N; oldest ends up first.
    static void sortAscending(int* indices, std::size_t count);
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_CORE_LOG_ROTATION_POLICY_H
