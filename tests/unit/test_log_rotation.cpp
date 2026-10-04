/**
 * test_log_rotation.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for log index allocation and the auto-purge policy.
 *
 * These are the flash-logging paths that are effectively unreachable on
 * hardware. Reaching the purge branch means filling a 1.9 MB partition;
 * reaching index exhaustion means creating a thousand files. Both are also
 * exactly the paths that fail on a long flying day rather than on the bench —
 * the purge runs at startLogging(), so a failure there means the flight is not
 * recorded at all, and nobody finds out until they go looking for the log.
 */
#include <gtest/gtest.h>

#include <vector>

#include "src/core/LogRotationPolicy.h"

using arduflite::drivers::LogRotationPolicy;

namespace {

int nextIndex(std::vector<int> indices)
{
    return LogRotationPolicy::nextIndex(indices.data(), indices.size());
}

// ── Index allocation ────────────────────────────────────────────────────────

TEST(LogRotation, EmptyDirectoryStartsAtZero)
{
    EXPECT_EQ(nextIndex({}), 0);
}

TEST(LogRotation, IndicesGrowMonotonically)
{
    EXPECT_EQ(nextIndex({ 0, 1, 2 }), 3);
    EXPECT_EQ(nextIndex({ 2, 0, 1 }), 3) << "order of the listing must not matter";
}

/**
 * Deleting a mid-range log does NOT reclaim its index while room remains above.
 * That is deliberate: it keeps log_007 reliably older than log_008, which is
 * what anyone reading a directory listing assumes. Reusing gaps eagerly would
 * make the numbering meaningless as an ordering.
 */
TEST(LogRotation, GapsAreNotReusedWhileRoomRemains)
{
    EXPECT_EQ(nextIndex({ 0, 1, 3, 4 }), 5) << "index 2 is free but must not be reused";
}

TEST(LogRotation, LowestGapIsReusedOnlyWhenTheTopIndexExists)
{
    std::vector<int> full;
    for (int i = 0; i <= 999; ++i) { full.push_back(i); }

    // Free index 4 only. With 999 present, the gap must now be reused.
    full.erase(full.begin() + 4);
    EXPECT_EQ(nextIndex(full), 4);
}

TEST(LogRotation, ExhaustedIndexSpaceReportsFailure)
{
    std::vector<int> full;
    for (int i = 0; i <= 999; ++i) { full.push_back(i); }

    EXPECT_EQ(nextIndex(full), -1)
        << "must report exhaustion, not silently overwrite log_000";
}

TEST(LogRotation, OutOfRangeEntriesAreIgnored)
{
    // A stray file parsing to an absurd index must not push allocation past the
    // ceiling, nor index a bitmap out of bounds.
    EXPECT_EQ(nextIndex({ 0, 1, 5000, -3 }), 2);
}

// ── Purge ───────────────────────────────────────────────────────────────────

TEST(LogRotation, NoPurgeWhenSpaceIsAmple)
{
    LogRotationPolicy policy;
    const int logs[] = { 0, 1, 2 };

    EXPECT_EQ(policy.purgeCount(logs, 3, 1000u * 1024u, 100u * 1024u), 0u);
}

TEST(LogRotation, PurgesJustEnoughToClearTheThreshold)
{
    LogRotationPolicy policy;          // minFreeBytes = 300 KB
    const int logs[] = { 0, 1, 2, 3, 4 };

    // 50 KB free, 100 KB reclaimed per log -> need 250 KB -> 3 deletions.
    EXPECT_EQ(policy.purgeCount(logs, 5, 50u * 1024u, 100u * 1024u), 3u);
}

TEST(LogRotation, PurgeStopsAtTheNumberOfLogsAvailable)
{
    LogRotationPolicy policy;
    const int logs[] = { 0, 1 };

    // Two logs cannot free 300 KB at 10 KB each; it must not report more
    // deletions than there are files.
    EXPECT_EQ(policy.purgeCount(logs, 2, 0, 10u * 1024u), 2u);
}

/**
 * A corrupt filesystem can report free space that never rises no matter what is
 * deleted. Without a cap the purge loop would delete every log on the device
 * chasing a threshold it can never reach — destroying the flight history while
 * trying to make room for one more flight.
 */
TEST(LogRotation, PurgeIsCappedAgainstAFilesystemThatNeverFreesSpace)
{
    LogRotationPolicy policy;
    policy.maxPurgeAttempts = 5;

    std::vector<int> many;
    for (int i = 0; i < 100; ++i) { many.push_back(i); }

    EXPECT_EQ(policy.purgeCount(many.data(), many.size(), 0, 1u), 5u)
        << "must stop at the attempt cap, not delete all 100 logs";
}

TEST(LogRotation, ZeroReclaimPerLogPurgesNothing)
{
    LogRotationPolicy policy;
    const int logs[] = { 0, 1, 2 };

    // A store reporting that deleting a log frees nothing. Deleting anyway
    // destroys data for no benefit.
    EXPECT_EQ(policy.purgeCount(logs, 3, 0, 0), 0u);
}

TEST(LogRotation, ExactlyAtTheThresholdDoesNotPurge)
{
    LogRotationPolicy policy;
    const int logs[] = { 0, 1 };

    EXPECT_EQ(policy.purgeCount(logs, 2, 300u * 1024u, 100u * 1024u), 0u)
        << "the threshold is a floor, not a trigger point";
}

// ── Ordering ────────────────────────────────────────────────────────────────

TEST(LogRotation, SortPutsOldestFirst)
{
    int indices[] = { 7, 2, 9, 0, 5 };
    LogRotationPolicy::sortAscending(indices, 5);

    EXPECT_EQ(indices[0], 0);
    EXPECT_EQ(indices[4], 9);
}

TEST(LogRotation, SortHandlesEmptyAndSingle)
{
    int one[] = { 42 };
    LogRotationPolicy::sortAscending(one, 1);
    EXPECT_EQ(one[0], 42);

    LogRotationPolicy::sortAscending(nullptr, 0);   // must not fault
}

} // namespace
