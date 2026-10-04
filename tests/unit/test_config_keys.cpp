/**
 * test_config_keys.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Every config key the flight code COMPOSES must exist in the schema.
 *
 * This test exists because two did not, for seven phases.
 *
 * Commit 5c0a907 ("unit suffixes on every unit-bearing config key") renamed 45
 * key strings and missed `ConfigHelpers::buildPIDConfig()`, which builds its
 * keys with snprintf rather than using the macros. The result:
 *
 *   - `%s.ti` / `%s.td` no longer matched `…ti_s` / `…td_s`, so EVERY PID in
 *     both loops read 0 for its integral and derivative time constants and ran
 *     as a pure-P controller.
 *   - `%s.outlimit` no longer matched the attitude loop's `…outlimit_dps`, so
 *     the attitude PIDs clamped their own output to zero — ATTITUDE_MODE
 *     commanded no rate at all.
 *
 * Neither failed loudly. `ConfigRegistry::get()` logs a warning and returns a
 * default, the PID accepts it, and every other test passed. **A gain of zero
 * and a correctly-configured gain are indistinguishable except in flight.**
 *
 * A composed key is invisible to grep, which is why the rename missed it and
 * why this test asserts resolution rather than spelling.
 */
#include <gtest/gtest.h>

#include "include/ConfigKeys.h"
#include "src/utils/ConfigHelpers.h"
#include "src/utils/ConfigRegistry.h"

namespace {

class ConfigKeyTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        // Drains the schema's static registrations, as arduflite_init() does.
        ConfigRegistry::instance().init();
    }
};

/// Every PID prefix the controllers actually pass to buildPIDConfig().
struct PidLoop { const char* prefix; const char* outLimitSuffix; const char* label; };

const PidLoop kLoops[] = {
    { CONFIG_KEY_RATE_ROLL_PREFIX,  "outlimit",     "rate.roll"  },
    { CONFIG_KEY_RATE_PITCH_PREFIX, "outlimit",     "rate.pitch" },
    { CONFIG_KEY_RATE_YAW_PREFIX,   "outlimit",     "rate.yaw"   },
    { CONFIG_KEY_ATT_ROLL_PREFIX,   "outlimit_dps", "att.roll"   },
    { CONFIG_KEY_ATT_PITCH_PREFIX,  "outlimit_dps", "att.pitch"  },
    { CONFIG_KEY_ATT_YAW_PREFIX,    "outlimit_dps", "att.yaw"    },
};

/**
 * The output limit is the one that matters most: a PID clamps its own output to
 * it, so a missing key does not degrade the loop — it switches the loop OFF,
 * while every gain beside it still reads correctly.
 */
TEST_F(ConfigKeyTest, EveryLoopResolvesANonZeroOutputLimit)
{
    for (const auto& loop : kLoops)
    {
        const PIDConfig config = ConfigHelpers::buildPIDConfig(loop.prefix, loop.outLimitSuffix);
        EXPECT_GT(config.outLimit, 0.0f)
            << loop.label << ": output limit resolved to zero - the key is missing "
               "from the schema, and this loop commands nothing";
    }
}

TEST_F(ConfigKeyTest, EveryLoopResolvesANonZeroProportionalGain)
{
    for (const auto& loop : kLoops)
    {
        const PIDConfig config = ConfigHelpers::buildPIDConfig(loop.prefix, loop.outLimitSuffix);
        EXPECT_GT(config.kp, 0.0f) << loop.label << ": kp resolved to zero";
    }
}

/**
 * The precise property: what buildPIDConfig() COMPOSES must resolve to the same
 * value the macro-spelled key does.
 *
 * Value-based rather than "is it non-zero", because a zero can be legitimate —
 * anyone tuning the aircraft may deliberately set an integral term to 0.
 * Comparing against the macro-spelled key is what distinguishes "the composer
 * spelled it wrong and silently got a default" from "the schema really says
 * zero".
 */
TEST_F(ConfigKeyTest, ComposedKeysResolveToTheSameValuesAsTheMacroKeys)
{
    auto& reg = ConfigRegistry::instance();

    struct Expected { const char* prefix; const char* outSuffix;
                      const char* kpKey; const char* outKey; const char* label; };

    const Expected kExpected[] = {
        { CONFIG_KEY_RATE_ROLL_PREFIX,  "outlimit",     CONFIG_KEY_RATE_ROLL_KP,  CONFIG_KEY_RATE_ROLL_OUTLIMIT,      "rate.roll"  },
        { CONFIG_KEY_RATE_PITCH_PREFIX, "outlimit",     CONFIG_KEY_RATE_PITCH_KP, CONFIG_KEY_RATE_PITCH_OUTLIMIT,     "rate.pitch" },
        { CONFIG_KEY_RATE_YAW_PREFIX,   "outlimit",     CONFIG_KEY_RATE_YAW_KP,   CONFIG_KEY_RATE_YAW_OUTLIMIT,       "rate.yaw"   },
        { CONFIG_KEY_ATT_ROLL_PREFIX,   "outlimit_dps", CONFIG_KEY_ATT_ROLL_KP,   CONFIG_KEY_ATT_ROLL_OUTLIMIT_DPS,   "att.roll"   },
        { CONFIG_KEY_ATT_PITCH_PREFIX,  "outlimit_dps", CONFIG_KEY_ATT_PITCH_KP,  CONFIG_KEY_ATT_PITCH_OUTLIMIT_DPS,  "att.pitch"  },
    };

    for (const auto& e : kExpected)
    {
        const PIDConfig config = ConfigHelpers::buildPIDConfig(e.prefix, e.outSuffix);

        EXPECT_FLOAT_EQ(config.kp, reg.get<float>(e.kpKey))
            << e.label << ": composed kp disagrees with the macro key";
        EXPECT_FLOAT_EQ(config.outLimit, reg.get<float>(e.outKey))
            << e.label << ": composed output limit disagrees with the macro key - "
               "this is the failure that switched ATTITUDE_MODE off entirely";
    }
}

/// Roll and pitch DO carry integral and derivative time in the schema, so a
/// zero there means the '_s' suffix was lost.
TEST_F(ConfigKeyTest, RollAndPitchRateLoopsKeepTheirIandDTerms)
{
    for (const char* prefix : { CONFIG_KEY_RATE_ROLL_PREFIX, CONFIG_KEY_RATE_PITCH_PREFIX })
    {
        const PIDConfig config = ConfigHelpers::buildPIDConfig(prefix, "outlimit");
        EXPECT_GT(config.ki, 0.0f) << prefix << ": ki is zero - check the '_s' suffix on ti";
        EXPECT_GT(config.kd, 0.0f) << prefix << ": kd is zero - check the '_s' suffix on td";
    }
}

/// The registry must distinguish "absent" from "present and zero", or the
/// tests above could never tell a missing key from a deliberate zero.
TEST_F(ConfigKeyTest, AKeyThatDoesNotExistIsReportedAsMissing)
{
    EXPECT_FALSE(ConfigRegistry::instance().getParam("rate.roll.ti").has_value())
        << "the pre-rename spelling must NOT resolve - if it does, the schema "
           "has both and the rename was incomplete in the other direction";
    EXPECT_TRUE(ConfigRegistry::instance().getParam(CONFIG_KEY_RATE_ROLL_TI_S).has_value());
}

} // namespace
