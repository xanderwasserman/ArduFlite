/**
 * test_result.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "src/hal/core/Result.h"
#include "src/hal/core/Status.h"

using arduflite::Result;
using arduflite::Status;

TEST(Status, ToStringCoversEveryEnumerator)
{
    // If an enumerator is added without a toString case, this catches it.
    EXPECT_STREQ(toString(Status::Ok),             "Ok");
    EXPECT_STREQ(toString(Status::NotPresent),     "NotPresent");
    EXPECT_STREQ(toString(Status::IoError),        "IoError");
    EXPECT_STREQ(toString(Status::Timeout),        "Timeout");
    EXPECT_STREQ(toString(Status::InvalidArg),     "InvalidArg");
    EXPECT_STREQ(toString(Status::NotSupported),   "NotSupported");
    EXPECT_STREQ(toString(Status::NotInitialised), "NotInitialised");
    EXPECT_STREQ(toString(Status::Busy),           "Busy");
    EXPECT_STREQ(toString(Status::OutOfRange),     "OutOfRange");
    EXPECT_STREQ(toString(Status::Corrupt),        "Corrupt");
    EXPECT_STREQ(toString(Status::NoSpace),        "NoSpace");
}

TEST(Result, HoldsAValue)
{
    const Result<int> r{ 42 };
    EXPECT_TRUE(r.ok());
    EXPECT_TRUE(static_cast<bool>(r));
    EXPECT_EQ(r.status(), Status::Ok);
    EXPECT_EQ(r.value(), 42);
}

TEST(Result, HoldsAnError)
{
    const Result<int> r{ Status::IoError };
    EXPECT_FALSE(r.ok());
    EXPECT_FALSE(static_cast<bool>(r));
    EXPECT_EQ(r.status(), Status::IoError);
    EXPECT_EQ(r.valueOr(-1), -1);
}

TEST(Result, ValueOrReturnsValueWhenOk)
{
    const Result<int> r{ 7 };
    EXPECT_EQ(r.valueOr(-1), 7);
}

TEST(Result, IsConstexprFriendly)
{
    constexpr Result<int> ok{ 5 };
    static_assert(ok.ok());
    static_assert(ok.value() == 5);

    constexpr Result<int> bad{ Status::Timeout };
    static_assert(!bad.ok());
    static_assert(bad.status() == Status::Timeout);
    SUCCEED();
}

TEST(Result, MovesOutOfATemporary)
{
    auto make = [] { return Result<std::string>{ std::string(64, 'x') }; };
    const std::string s = make().value();
    EXPECT_EQ(s.size(), 64u);
}

TEST(Result, PointerPayload)
{
    int  target = 3;
    const Result<int*> r{ &target };
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(*r.value(), 3);

    const Result<int*> bad{ Status::NotPresent };
    EXPECT_FALSE(bad.ok());
    EXPECT_EQ(bad.valueOr(nullptr), nullptr);
}

namespace {
Status alwaysFails() { return Status::Corrupt; }
Status alwaysOk()    { return Status::Ok; }

Status runsBoth(int& counter)
{
    ARDUFLITE_TRY(alwaysOk());
    ++counter;
    ARDUFLITE_TRY(alwaysFails());
    ++counter;                 // must NOT be reached
    return Status::Ok;
}
} // namespace

TEST(ArdufliteTry, ShortCircuitsAndPreservesStatus)
{
    int counter = 0;
    EXPECT_EQ(runsBoth(counter), Status::Corrupt);
    EXPECT_EQ(counter, 1);
}
