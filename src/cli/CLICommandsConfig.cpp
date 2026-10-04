/**
 * CLICommandsConfig.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/CLICommands.h"
#include "src/cli/CLICommandContext.h"
#include "src/cli/CLICommandUtils.h"
#include "src/utils/ConfigPersistence.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/Logging.h"
#include "include/ConfigKeys.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Glob-style pattern matching for config keys.
 *
 * Iterative O(N+M) algorithm — avoids the exponential recursion of a naive
 * recursive implementation on pathological patterns like "a*a*a*b".
 *
 * Supports:
 *   "*"           - matches everything
 *   "rate.*"      - matches rate.roll.kp, rate.pitch.ti, etc.
 *   "rate.*.kp"   - matches rate.roll.kp, rate.pitch.kp, etc.
 *   "rate.roll.*" - matches rate.roll.kp, rate.roll.ti, etc.
 */
static bool matchGlob(const char* pattern, const char* key)
{
    const char* starP  = nullptr; // position of the last '*' in pattern
    const char* starK  = key;     // position in key when the last '*' was matched

    while (*key)
    {
        if (*pattern == '*')
        {
            starP = pattern++;
            starK = key;
        }
        else if (*pattern == *key)
        {
            pattern++;
            key++;
        }
        else if (starP)
        {
            // Backtrack: advance past one more key char and retry after last '*'
            pattern = starP + 1;
            key     = ++starK;
        }
        else
        {
            return false;
        }
    }

    // Consume any trailing wildcards
    while (*pattern == '*') pattern++;

    return *pattern == '\0';
}

static std::vector<std::string> matchingConfigKeys(
    const std::unordered_map<std::string, ConfigParam>& params,
    const String& pattern)
{
    std::vector<std::string> keys;
    for (const auto& kv : params)
    {
        if (matchGlob(pattern.c_str(), kv.first.c_str()))
        {
            keys.push_back(kv.first);
        }
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

static void logConfigListValue(const ConfigParam& param)
{
    switch (param.type)
    {
        case ConfigType::FLOAT:
            LOG("  %s = %.4f", param.key, param.currentVal.f);
            break;
        case ConfigType::INT32:
            LOG("  %s = %d", param.key, param.currentVal.i);
            break;
        case ConfigType::UINT8:
            LOG("  %s = %u", param.key, param.currentVal.u8);
            break;
        case ConfigType::BOOL:
            LOG("  %s = %s", param.key, param.currentVal.b ? "true" : "false");
            break;
        case ConfigType::STRING:
            LOG("  %s = \"%s\"", param.key, param.currentVal.s);
            break;
    }
}

static void logConfigDetailValue(const ConfigParam& param)
{
    switch (param.type)
    {
        case ConfigType::FLOAT:
            LOG("%s = %.4f (default: %.4f, range: %.4f - %.4f)",
                param.key, param.currentVal.f, param.defaultVal.f, param.minVal.f, param.maxVal.f);
            break;
        case ConfigType::INT32:
            LOG("%s = %d (default: %d, range: %d - %d)",
                param.key, param.currentVal.i, param.defaultVal.i, param.minVal.i, param.maxVal.i);
            break;
        case ConfigType::UINT8:
            LOG("%s = %u (default: %u, range: %u - %u)",
                param.key, param.currentVal.u8, param.defaultVal.u8, param.minVal.u8, param.maxVal.u8);
            break;
        case ConfigType::BOOL:
            LOG("%s = %s (default: %s)",
                param.key, param.currentVal.b ? "true" : "false", param.defaultVal.b ? "true" : "false");
            break;
        case ConfigType::STRING:
            LOG("%s = \"%s\" (default: \"%s\")",
                param.key, param.currentVal.s, param.defaultVal.s);
            break;
    }
    LOG("  Description: %s", param.description);
}

static void logConfigRangeValue(const ConfigParam& param)
{
    switch (param.type)
    {
        case ConfigType::FLOAT:
            LOG("%s = %.4f (range: %.4f - %.4f)",
                param.key, param.currentVal.f, param.minVal.f, param.maxVal.f);
            break;
        case ConfigType::INT32:
            LOG("%s = %d (range: %d - %d)",
                param.key, param.currentVal.i, param.minVal.i, param.maxVal.i);
            break;
        case ConfigType::UINT8:
            LOG("%s = %u (range: %u - %u)",
                param.key, param.currentVal.u8, param.minVal.u8, param.maxVal.u8);
            break;
        case ConfigType::BOOL:
            LOG("%s = %s", param.key, param.currentVal.b ? "true" : "false");
            break;
        case ConfigType::STRING:
            LOG("%s = \"%s\"", param.key, param.currentVal.s);
            break;
    }
}

static void logConfigHelp()
{
    LOG("Unknown config command. Available:");
    LOG("  config list [pattern] → list params (e.g., 'rate.*')");
    LOG("  config get <key>      → show param details");
    LOG("  config set <key> <val>→ set param value");
    LOG("  config save           → save dirty params to NVS");
    LOG("  config load           → reload from NVS");
    LOG("  config defaults       → reset to factory defaults");
}

static bool setConfigValueFromString(
    ConfigRegistry& reg,
    const ConfigParam& param,
    const String& key,
    const String& valueText)
{
    switch (param.type)
    {
        case ConfigType::FLOAT:
        {
            float value = 0.0f;
            if (!parseFloatStrict(valueText, value))
            {
                LOG_ERR("Invalid float value for %s: %s", key.c_str(), valueText.c_str());
                return false;
            }
            return reg.set<float>(key.c_str(), value);
        }
        case ConfigType::INT32:
        {
            int32_t value = 0;
            if (!parseIntStrict(valueText, value))
            {
                LOG_ERR("Invalid integer value for %s: %s", key.c_str(), valueText.c_str());
                return false;
            }
            return reg.set<int32_t>(key.c_str(), value);
        }
        case ConfigType::UINT8:
        {
            uint8_t value = 0;
            if (!parseUInt8Strict(valueText, value))
            {
                LOG_ERR("Invalid uint8 value for %s: %s", key.c_str(), valueText.c_str());
                return false;
            }
            return reg.set<uint8_t>(key.c_str(), value);
        }
        case ConfigType::BOOL:
        {
            bool value = false;
            if (!parseBoolStrict(valueText, value))
            {
                LOG_ERR("Invalid bool value for %s: use true/false, yes/no, on/off, or 1/0", key.c_str());
                return false;
            }
            return reg.set<bool>(key.c_str(), value);
        }
        case ConfigType::STRING:
            return reg.set<std::string>(key.c_str(), std::string(valueText.c_str()));
    }

    return false;
}

void cmdConfig(const String &args)
{
    auto& reg = ConfigRegistry::instance();

    ParsedCommand parsed = parseCommandArgs(args);
    const String& cmd = parsed.command;
    const String& remainder = parsed.remainder;

    if (cmd == "list" || cmd.isEmpty())
    {
        String pattern = remainder.isEmpty() ? "*" : remainder;
        auto params = reg.getAllParams();
        std::vector<std::string> keys = matchingConfigKeys(params, pattern);

        for (const auto& key : keys)
        {
            logConfigListValue(params[key]);
        }
        LOG("(%u parameter%s)", (unsigned)keys.size(), keys.size() == 1 ? "" : "s");
    }
    else if (cmd == "get")
    {
        if (remainder.isEmpty())
        {
            LOG_ERR("Usage: config get <key|pattern>");
            return;
        }

        const bool isPattern = remainder.indexOf('*') >= 0;
        if (!isPattern)
        {
            auto optParam = reg.getParam(remainder.c_str());
            if (!optParam)
            {
                LOG_ERR("Unknown key: %s", remainder.c_str());
                return;
            }

            logConfigDetailValue(*optParam);
        }
        else
        {
            auto params = reg.getAllParams();
            std::vector<std::string> keys = matchingConfigKeys(params, remainder);

            if (keys.empty())
            {
                LOG_ERR("No keys matching: %s", remainder.c_str());
                return;
            }

            for (const auto& key : keys)
            {
                logConfigRangeValue(params[key]);
            }
            LOG("(%u parameter%s)", (unsigned)keys.size(), keys.size() == 1 ? "" : "s");
        }
    }
    else if (cmd == "set")
    {
        if (rejectUnsafeGroundCommand("change configuration")) return;

        const int valIdx = remainder.indexOf(' ');
        if (valIdx <= 0)
        {
            LOG_ERR("Usage: config set <key> <value>");
            return;
        }

        String key = remainder.substring(0, valIdx);
        String valStr = remainder.substring(valIdx + 1);
        key.trim();
        valStr.trim();

        auto optParam = reg.getParam(key.c_str());
        if (!optParam)
        {
            LOG_ERR("Unknown key: %s", key.c_str());
            return;
        }

        if (setConfigValueFromString(reg, *optParam, key, valStr))
        {
            LOG("Set %s OK", key.c_str());
        }
        else
        {
            LOG_ERR("Failed to set %s (value out of range?)", key.c_str());
        }
    }
    else if (cmd == "save")
    {
        size_t saved = ConfigPersistence::saveIfDirty();
        LOG("Saved %u parameter(s) to NVS", saved);
    }
    else if (cmd == "load")
    {
        if (rejectUnsafeGroundCommand("load configuration")) return;

        ConfigPersistence::load();
        LOG("Loaded configuration from NVS");
    }
    else if (cmd == "defaults")
    {
        if (rejectUnsafeGroundCommand("reset configuration")) return;

        reg.resetAll();
        LOG("Reset all parameters to defaults (not saved yet)");
    }
    else
    {
        logConfigHelp();
    }
}
