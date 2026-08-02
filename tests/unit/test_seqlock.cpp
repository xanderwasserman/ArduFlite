/**
 * test_seqlock.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * Tests for arduflite::SeqLock — generalised from the ArduFliteIMU snapshot.
 * The concurrency test is the point: the in-place original could not be tested.
 */
#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "src/hal/core/SeqLock.h"

using arduflite::SeqLock;

namespace {

/// Deliberately wide, with a checksum, so a torn read is detectable.
struct Payload
{
    std::uint32_t seq = 0;
    float         a[8]{};
    std::uint32_t checksum = 0;

    static Payload make(std::uint32_t s)
    {
        Payload p;
        p.seq = s;
        std::uint32_t sum = s;
        for (int i = 0; i < 8; ++i)
        {
            p.a[i] = static_cast<float>(s + static_cast<std::uint32_t>(i));
            sum += static_cast<std::uint32_t>(p.a[i]);
        }
        p.checksum = sum;
        return p;
    }

    [[nodiscard]] bool coherent() const
    {
        std::uint32_t sum = seq;
        for (int i = 0; i < 8; ++i) { sum += static_cast<std::uint32_t>(a[i]); }
        return sum == checksum;
    }
};

} // namespace

TEST(SeqLock, ReadsWhatWasPublished)
{
    SeqLock<Payload> lock;
    lock.publish(Payload::make(42));

    const Payload got = lock.read();
    EXPECT_EQ(got.seq, 42u);
    EXPECT_TRUE(got.coherent());
}

TEST(SeqLock, DefaultBeforeFirstPublishIsCoherent)
{
    SeqLock<Payload> lock;
    const Payload got = lock.read();
    EXPECT_EQ(got.seq, 0u);
    EXPECT_TRUE(got.coherent());
}

TEST(SeqLock, TryReadReturnsTrueWhenUncontended)
{
    SeqLock<Payload> lock;
    lock.publish(Payload::make(7));

    Payload out{};
    EXPECT_TRUE(lock.tryRead(out));
    EXPECT_EQ(out.seq, 7u);
}

TEST(SeqLock, HealthCountersStartClean)
{
    SeqLock<Payload> lock;
    lock.publish(Payload::make(1));
    (void)lock.read();

    const auto h = lock.health();
    EXPECT_EQ(h.totalRetries,   0u);
    EXPECT_EQ(h.maxRetries,     0u);
    EXPECT_EQ(h.retryLimitHits, 0u);
}

TEST(SeqLock, ConcurrentReadsAreAlwaysCoherent)
{
    // This is the test the original in-place seqlock could not have.
    SeqLock<Payload> lock;
    lock.publish(Payload::make(0));

    std::atomic<bool> stop{ false };
    std::atomic<int>  tornReads{ 0 };
    std::atomic<int>  readCount{ 0 };

    std::thread writer([&] {
        for (std::uint32_t i = 1; !stop.load(std::memory_order_relaxed); ++i)
        {
            lock.publish(Payload::make(i));
        }
    });

    std::vector<std::thread> readers;
    readers.reserve(3);
    for (int r = 0; r < 3; ++r)
    {
        readers.emplace_back([&] {
            while (!stop.load(std::memory_order_relaxed))
            {
                const Payload p = lock.read();
                if (!p.coherent()) { tornReads.fetch_add(1, std::memory_order_relaxed); }
                readCount.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    stop.store(true, std::memory_order_relaxed);

    writer.join();
    for (auto& t : readers) { t.join(); }

    EXPECT_GT(readCount.load(), 1000) << "test did not exercise enough reads to mean anything";
    EXPECT_EQ(tornReads.load(), 0)    << "seqlock returned a torn payload";
}

TEST(SeqLock, RetryLimitFallbackStaysCoherent)
{
    // A retry limit of 1 makes the fallback path trivially reachable under
    // contention; the fallback must still be a coherent value.
    SeqLock<Payload, 1> lock;
    lock.publish(Payload::make(5));

    std::atomic<bool> stop{ false };
    std::atomic<int>  tornReads{ 0 };

    std::thread writer([&] {
        for (std::uint32_t i = 6; !stop.load(std::memory_order_relaxed); ++i)
        {
            lock.publish(Payload::make(i));
        }
    });

    for (int i = 0; i < 200000; ++i)
    {
        const Payload p = lock.read();
        if (!p.coherent()) { tornReads.fetch_add(1, std::memory_order_relaxed); }
    }

    stop.store(true, std::memory_order_relaxed);
    writer.join();

    EXPECT_EQ(tornReads.load(), 0) << "stale fallback returned a torn payload";
}
