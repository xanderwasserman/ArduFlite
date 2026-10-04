/**
 * test_key_value_store.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Contract tests for hal::KeyValueStore.
 *
 * ConfigPersistence moved onto this interface in Phase 7 (ADR-027), which
 * offers four byte-oriented accessors rather than fifteen typed ones.
 * The typed API made several decisions on the caller's behalf; the byte API
 * hands them back, and these tests pin the ones that matter.
 *
 * Run against MemoryKeyValueStore. Esp32KeyValueStore is written to the same
 * contract — the comments in each mirror the assertions here.
 */
#include <gtest/gtest.h>

#include <cstring>

#include "hal_host/HostPlatform.h"

using namespace arduflite;
using arduflite::hal::host::MemoryKeyValueStore;

namespace {

TEST(KeyValueStore, MissingKeyIsNotPresentNotAnError)
{
    MemoryKeyValueStore store;
    float value = 0.0f;
    std::size_t length = 0;

    EXPECT_EQ(store.read("absent", &value, sizeof(value), length), Status::NotPresent);
    EXPECT_EQ(length, 0u);
    EXPECT_EQ(value, 0.0f) << "a failed read must not scribble on the destination";
}

TEST(KeyValueStore, RoundTripsAValue)
{
    MemoryKeyValueStore store;
    const float written = 3.14159f;
    ASSERT_EQ(store.write("pi", &written, sizeof(written)), Status::Ok);

    float read = 0.0f;
    std::size_t length = 0;
    ASSERT_EQ(store.read("pi", &read, sizeof(read), length), Status::Ok);
    EXPECT_EQ(length, sizeof(float));
    EXPECT_FLOAT_EQ(read, written);
}

/**
 * A missing key is reported, never silently defaulted.
 *
 * A stored value larger than the caller's buffer must be REFUSED. Truncating
 * would hand back a different value that looks valid — for a float, four bytes
 * of an eight-byte double is not a smaller number, it is a wrong one.
 */
TEST(KeyValueStore, OversizedValueIsRefusedNotTruncated)
{
    MemoryKeyValueStore store;
    store.writeRaw("wide", { 1, 2, 3, 4, 5, 6, 7, 8 });

    std::uint8_t buffer[4]{};
    std::size_t length = 0;
    EXPECT_EQ(store.read("wide", buffer, sizeof(buffer), length), Status::NoSpace);
    EXPECT_EQ(length, 8u) << "the caller is told how much space it would need";
    EXPECT_EQ(buffer[0], 0) << "nothing may be copied on a refused read";
}

/**
 * ConfigPersistence relies on this to distinguish "never written" from "written
 * by a build where this key had a different type". Reinterpreting a 1-byte bool
 * as a 4-byte float would read three bytes of adjacent garbage and produce a
 * plausible-looking configuration value.
 */
TEST(KeyValueStore, LengthMismatchIsDetectable)
{
    MemoryKeyValueStore store;
    const bool storedAsBool = true;
    ASSERT_EQ(store.write("mode", &storedAsBool, sizeof(storedAsBool)), Status::Ok);

    float readAsFloat = 0.0f;
    std::size_t length = 0;
    const Status status = store.read("mode", &readAsFloat, sizeof(readAsFloat), length);

    ASSERT_EQ(status, Status::Ok) << "it fits, so the read itself succeeds";
    EXPECT_NE(length, sizeof(float))
        << "but the LENGTH is how the caller detects the type changed";
    EXPECT_EQ(length, sizeof(bool));
}

TEST(KeyValueStore, WriteOverwritesRatherThanAppending)
{
    MemoryKeyValueStore store;
    const std::int32_t first = 42;
    const std::int32_t second = -7;
    ASSERT_EQ(store.write("n", &first, sizeof(first)), Status::Ok);
    ASSERT_EQ(store.write("n", &second, sizeof(second)), Status::Ok);

    std::int32_t read = 0;
    std::size_t length = 0;
    ASSERT_EQ(store.read("n", &read, sizeof(read), length), Status::Ok);
    EXPECT_EQ(read, second);
    EXPECT_EQ(store.size(), 1u);
}

TEST(KeyValueStore, VariableLengthValuesKeepTheirExactLength)
{
    MemoryKeyValueStore store;

    // Strings are stored at their real length plus terminator, not padded to
    // the full CONFIG_STRING_MAX_LEN buffer.
    const char ssid[] = "wing";
    ASSERT_EQ(store.write("ssid", ssid, std::strlen(ssid) + 1), Status::Ok);

    char buffer[64]{};
    std::size_t length = 0;
    ASSERT_EQ(store.read("ssid", buffer, sizeof(buffer), length), Status::Ok);
    EXPECT_EQ(length, 5u) << "four characters plus the terminator";
    EXPECT_STREQ(buffer, "wing");
}

TEST(KeyValueStore, EraseRemovesOneKey)
{
    MemoryKeyValueStore store;
    const int value = 1;
    ASSERT_EQ(store.write("a", &value, sizeof(value)), Status::Ok);
    ASSERT_EQ(store.write("b", &value, sizeof(value)), Status::Ok);

    EXPECT_EQ(store.erase("a"), Status::Ok);
    EXPECT_FALSE(store.has("a"));
    EXPECT_TRUE(store.has("b"));

    EXPECT_EQ(store.erase("a"), Status::NotPresent) << "erasing twice is not an error state";
}

/// The operation `erase(key)` cannot express: a caller doing a factory reset
/// does not know every key an older firmware version may have left behind.
TEST(KeyValueStore, EraseAllWipesKeysTheCallerNeverKnewAbout)
{
    MemoryKeyValueStore store;
    const int value = 1;
    ASSERT_EQ(store.write("known", &value, sizeof(value)), Status::Ok);
    store.writeRaw("left_by_old_firmware", { 9, 9 });
    ASSERT_EQ(store.size(), 2u);

    EXPECT_EQ(store.eraseAll(), Status::Ok);
    EXPECT_EQ(store.size(), 0u);
}

/**
 * Both implementations must reject the same bad arguments the same way.
 *
 * This test crashed when first written, because the fake looked up a null key
 * in a std::map — constructing a std::string from nullptr, which is undefined
 * behaviour. The real Esp32KeyValueStore had guarded it all along. A fake that
 * behaves differently from the thing it stands in for turns every test using it
 * into a test of the wrong contract.
 */
TEST(KeyValueStore, RejectsBadArgumentsRatherThanFaulting)
{
    MemoryKeyValueStore store;
    std::size_t length = 0;
    float value = 0.0f;

    EXPECT_EQ(store.read(nullptr, &value, sizeof(value), length), Status::InvalidArg);
    EXPECT_EQ(store.read("k", nullptr, sizeof(value), length), Status::InvalidArg);
    EXPECT_EQ(store.write(nullptr, &value, sizeof(value)), Status::InvalidArg);
    EXPECT_EQ(store.write("k", nullptr, sizeof(value)), Status::InvalidArg);
    EXPECT_EQ(store.erase(nullptr), Status::InvalidArg);

    // Zero length is rejected too: a key present with no bytes cannot be
    // distinguished from an absent one on read.
    EXPECT_EQ(store.write("k", &value, 0), Status::InvalidArg);
}

} // namespace
