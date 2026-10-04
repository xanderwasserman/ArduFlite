/**
 * test_mavlink_params.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * The MAVLink parameter table must track the configuration schema exactly: a
 * key without a row is invisible to a ground station, and a name that drops its
 * unit suffix undoes the convention the keys follow.
 */
#include <gtest/gtest.h>

#include <set>
#include <string>
#include <string_view>

#include "include/ConfigKeys.h"
#include "src/telemetry/mavlink/MavlinkParams.h"
#include "src/utils/ConfigRegistry.h"

using arduflite::mavlink::kParamIdLength;
using arduflite::mavlink::ParamWrite;
using arduflite::mavlink::paramNames;
using arduflite::mavlink::readParam;
using arduflite::mavlink::writeParam;

namespace {

class MavlinkParamsTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { ConfigRegistry::instance().init(); }
};

bool endsWith(std::string_view text, std::string_view suffix)
{
    return text.size() >= suffix.size() && text.substr(text.size() - suffix.size()) == suffix;
}

} // namespace

TEST_F(MavlinkParamsTest, EveryNumericKeyHasExactlyOneShortUniqueName)
{
    std::set<std::string> tableKeys;
    std::set<std::string> ids;
    for (const auto& name : paramNames())
    {
        EXPECT_TRUE(tableKeys.insert(name.key).second) << name.key << " appears twice";
        EXPECT_TRUE(ids.insert(name.id).second) << name.id << " is used twice";
        EXPECT_LE(std::string_view(name.id).size(), kParamIdLength) << name.id;
    }

    std::set<std::string> numericKeys;
    for (const ConfigParam& param : ConfigRegistry::instance().list("*"))
    {
        if (param.type != ConfigType::STRING) { numericKeys.insert(param.key); }
    }
    EXPECT_EQ(tableKeys, numericKeys);
}

TEST_F(MavlinkParamsTest, NamesKeepTheKeysUnitSuffix)
{
    struct Unit { std::string_view key; std::string_view id; };
    constexpr Unit kUnits[] = {
        { "_per_s", "_PER_S" }, { "_deg", "_DEG" }, { "_dps", "_DPS" }, { "_rad", "_RAD" },
        { "_us", "_US" }, { "_ms", "_MS" }, { "_pct", "_PCT" }, { "_bps", "_BPS" }, { "_g", "_G" },
        { "_s", "_S" },
    };

    for (const auto& name : paramNames())
    {
        for (const Unit& unit : kUnits)
        {
            if (endsWith(name.key, unit.key))
            {
                EXPECT_TRUE(endsWith(name.id, unit.id)) << name.key << " -> " << name.id;
                break;
            }
        }
    }
}

TEST_F(MavlinkParamsTest, WritesAreValidatedByTheRegistry)
{
    auto& registry = ConfigRegistry::instance();

    EXPECT_EQ(writeParam("RATE_RLL_KP", 0.25f), ParamWrite::Applied);
    EXPECT_FLOAT_EQ(readParam("RATE_RLL_KP")->value, 0.25f);

    EXPECT_EQ(writeParam("RATE_RLL_KP", 1.0e6f), ParamWrite::Rejected);
    EXPECT_FLOAT_EQ(readParam("RATE_RLL_KP")->value, 0.25f) << "a rejected write must leave the old value";

    EXPECT_EQ(writeParam("MAV_UART_BAUD", 57600.5f), ParamWrite::Rejected) << "integer parameter";
    EXPECT_EQ(writeParam("MAV_UART_ENABLED", 2.0f), ParamWrite::Rejected) << "boolean parameter";
    EXPECT_EQ(writeParam("NO_SUCH_PARAM", 1.0f), ParamWrite::Unknown);

    registry.reset(CONFIG_KEY_RATE_ROLL_KP);
}
