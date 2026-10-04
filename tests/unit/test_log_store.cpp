/**
 * test_log_store.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the device::LogStore session contract.
 *
 * Driven against MemoryLogStore. The contract these pin is the one
 * LittleFsLogStore must also honour — in particular what happens when the
 * medium fills mid-flight, which on hardware requires filling a 1.9 MB
 * partition to reach.
 */
#include <gtest/gtest.h>

#include "hal_host/HostPlatform.h"

using namespace arduflite;
using arduflite::hal::host::MemoryLogStore;

namespace {

class LogStoreTest : public ::testing::Test
{
protected:
    void SetUp() override { ASSERT_EQ(store.begin(), Status::Ok); }
    MemoryLogStore store;
};

// ── Session lifecycle ───────────────────────────────────────────────────────

TEST_F(LogStoreTest, WriteThenReadBackRoundTrips)
{
    ASSERT_EQ(store.openSession(3), Status::Ok);
    ASSERT_EQ(store.append("hello,", 6), Status::Ok);
    ASSERT_EQ(store.append("world", 5), Status::Ok);
    ASSERT_EQ(store.closeSession(), Status::Ok);

    char buffer[32]{};
    std::size_t length = 0;
    ASSERT_EQ(store.readSession(3, buffer, sizeof(buffer), 0, length), Status::Ok);
    EXPECT_EQ(length, 11u);
    EXPECT_EQ(std::string(buffer, length), "hello,world");
}

TEST_F(LogStoreTest, AppendWithoutAnOpenSessionIsRejected)
{
    EXPECT_EQ(store.append("data", 4), Status::NotPresent)
        << "silently discarding rows would lose a flight without saying so";
}

TEST_F(LogStoreTest, OnlyOneSessionMayBeOpenAtATime)
{
    ASSERT_EQ(store.openSession(0), Status::Ok);
    EXPECT_EQ(store.openSession(1), Status::Busy);
}

TEST_F(LogStoreTest, ReopeningAnIndexTruncatesIt)
{
    ASSERT_EQ(store.openSession(5), Status::Ok);
    ASSERT_EQ(store.append("old data", 8), Status::Ok);
    ASSERT_EQ(store.closeSession(), Status::Ok);

    ASSERT_EQ(store.openSession(5), Status::Ok);
    ASSERT_EQ(store.append("new", 3), Status::Ok);
    ASSERT_EQ(store.closeSession(), Status::Ok);

    char buffer[32]{};
    std::size_t length = 0;
    ASSERT_EQ(store.readSession(5, buffer, sizeof(buffer), 0, length), Status::Ok);
    EXPECT_EQ(std::string(buffer, length), "new")
        << "an index must not accumulate two flights' rows";
}

TEST_F(LogStoreTest, ReadingAMissingSessionReportsNotPresent)
{
    std::size_t length = 0;
    char buffer[8]{};
    EXPECT_EQ(store.readSession(42, buffer, sizeof(buffer), 0, length), Status::NotPresent);
    EXPECT_EQ(length, 0u);
}

/// Streaming a log too large for any single buffer — how `dumplog` works.
TEST_F(LogStoreTest, ReadsAtAnOffsetToStreamALargeSession)
{
    ASSERT_EQ(store.openSession(1), Status::Ok);
    ASSERT_EQ(store.append("ABCDEFGHIJ", 10), Status::Ok);
    ASSERT_EQ(store.closeSession(), Status::Ok);

    std::string assembled;
    char chunk[4]{};
    std::size_t offset = 0;
    for (;;)
    {
        std::size_t length = 0;
        ASSERT_EQ(store.readSession(1, chunk, sizeof(chunk), offset, length), Status::Ok);
        if (length == 0) { break; }
        assembled.append(chunk, length);
        offset += length;
    }

    EXPECT_EQ(assembled, "ABCDEFGHIJ");
}

TEST_F(LogStoreTest, ReadingPastTheEndReturnsNothingRatherThanFailing)
{
    ASSERT_EQ(store.openSession(1), Status::Ok);
    ASSERT_EQ(store.append("data", 4), Status::Ok);
    ASSERT_EQ(store.closeSession(), Status::Ok);

    char buffer[8]{};
    std::size_t length = 1;
    EXPECT_EQ(store.readSession(1, buffer, sizeof(buffer), 100, length), Status::Ok)
        << "running off the end is how a streaming caller learns it is done";
    EXPECT_EQ(length, 0u);
}

TEST_F(LogStoreTest, ReadTruncatesToTheCallersBuffer)
{
    ASSERT_EQ(store.openSession(1), Status::Ok);
    ASSERT_EQ(store.append("0123456789", 10), Status::Ok);
    ASSERT_EQ(store.closeSession(), Status::Ok);

    char small[4]{};
    std::size_t length = 0;
    ASSERT_EQ(store.readSession(1, small, sizeof(small), 0, length), Status::Ok);
    EXPECT_EQ(length, 4u) << "must report what it wrote, not what exists";
}

// ── Listing and removal ─────────────────────────────────────────────────────

TEST_F(LogStoreTest, ListsEverySessionPresent)
{
    for (std::uint16_t i : { 0, 7, 3 })
    {
        ASSERT_EQ(store.openSession(i), Status::Ok);
        ASSERT_EQ(store.closeSession(), Status::Ok);
    }

    std::uint16_t indices[8]{};
    EXPECT_EQ(store.listSessions(indices, 8), 3u);
}

TEST_F(LogStoreTest, ListRespectsTheCallersCap)
{
    for (std::uint16_t i = 0; i < 10; ++i)
    {
        ASSERT_EQ(store.openSession(i), Status::Ok);
        ASSERT_EQ(store.closeSession(), Status::Ok);
    }

    std::uint16_t indices[4]{};
    EXPECT_EQ(store.listSessions(indices, 4), 4u)
        << "must not write past the buffer it was given";
}

TEST_F(LogStoreTest, RemovingTheOpenSessionIsRefused)
{
    ASSERT_EQ(store.openSession(2), Status::Ok);
    EXPECT_EQ(store.removeSession(2), Status::Busy)
        << "deleting the file being written is implementation-defined on LittleFS";
}

TEST_F(LogStoreTest, RemoveReportsWhetherAnythingWasThere)
{
    ASSERT_EQ(store.openSession(1), Status::Ok);
    ASSERT_EQ(store.closeSession(), Status::Ok);

    EXPECT_EQ(store.removeSession(1), Status::Ok);
    EXPECT_EQ(store.removeSession(1), Status::NotPresent);
}

// ── The full-medium path ────────────────────────────────────────────────────

/**
 * The reason this file exists. A short write means the medium filled mid-flight;
 * the store must SAY so, because a caller that keeps appending regardless
 * produces a log that is silently truncated from that row on — and nobody
 * discovers it until they go looking for the end of the flight.
 */
TEST_F(LogStoreTest, AppendReportsNoSpaceWhenTheMediumIsFull)
{
    store.setCapacity(64);
    ASSERT_EQ(store.openSession(0), Status::Ok);

    const std::string row(32, 'a');
    ASSERT_EQ(store.append(row.data(), row.size()), Status::Ok);
    ASSERT_EQ(store.append(row.data(), row.size()), Status::Ok);

    EXPECT_EQ(store.append(row.data(), row.size()), Status::NoSpace);
}

TEST_F(LogStoreTest, FreeingSpaceMakesAppendsSucceedAgain)
{
    store.setCapacity(64);
    store.seedSession(9, 60);

    ASSERT_EQ(store.openSession(0), Status::Ok);
    const std::string row(16, 'b');
    ASSERT_EQ(store.append(row.data(), row.size()), Status::NoSpace);

    ASSERT_EQ(store.removeSession(9), Status::Ok);
    EXPECT_EQ(store.append(row.data(), row.size()), Status::Ok)
        << "this is the purge path: delete an old flight, then record the new one";
}

TEST_F(LogStoreTest, UsageReportsWhatIsStored)
{
    store.setCapacity(1000);
    store.seedSession(1, 250);
    store.seedSession(2, 100);

    std::uint32_t used = 0, total = 0;
    ASSERT_EQ(store.usage(used, total), Status::Ok);
    EXPECT_EQ(used, 350u);
    EXPECT_EQ(total, 1000u);
}

TEST_F(LogStoreTest, FormatAllClearsEverything)
{
    store.seedSession(1, 10);
    store.seedSession(2, 10);
    ASSERT_EQ(store.openSession(3), Status::Ok);

    ASSERT_EQ(store.formatAll(), Status::Ok);

    EXPECT_EQ(store.sessionCount(), 0u);
    EXPECT_FALSE(store.isOpen()) << "format must not leave a handle to a deleted file";
}

} // namespace
