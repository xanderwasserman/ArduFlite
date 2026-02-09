/**
 * CLICommandsConfig.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "CLICommandsConfig.h"
#include "src/utils/Logging.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/ConfigPersistence.h"
#include "include/ConfigKeys.h"
#include <vector>
#include <algorithm>

// Global pointer for the controller (accessible in command functions).
ArduFliteController* globalController = nullptr;
ArduFliteIMU* globalIMU = nullptr;
ArduFliteFlashTelemetry* globalFlashTelemetry = nullptr;

void setCliController(ArduFliteController* controller) 
{
    globalController = controller;
}

void setCliIMU(ArduFliteIMU* imu) 
{
    globalIMU = imu;
}

void setFlashTelemetry(ArduFliteFlashTelemetry* flashTelemetry) 
{
    globalFlashTelemetry = flashTelemetry;
}

// Command functions:
void cmdHelp(const String &args) 
{
    LOG("Available commands:");
    for (size_t i = 0; i < numCLICommands; i++)
     {
        LOG_N("  ");
        LOG_N("%s", cliCommands[i].command);
        LOG_N(" - ");
        LOG("%s", cliCommands[i].description);
    }
}

void cmdReset(const String &args) 
{
    LOG("Resetting system...");
    ESP.restart();
}

void cmdStats(const String &args) 
{
    if (!globalController) 
    {
        LOG_ERR("Controller not set!");
        return;
    }
    // Retrieve and print stats.
    LoopStats outerStats = globalController->getOuterLoopStats();
    LoopStats innerStats = globalController->getInnerLoopStats();

    LOG("Outer Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, percentage: %lu",
                  outerStats.avgDt, outerStats.maxDt, outerStats.overrunCount, (outerStats.overrunCount/outerStats.sampleCount)*100);
    LOG("Inner Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, percentage: %lu",
                  innerStats.avgDt, innerStats.maxDt, innerStats.overrunCount, (innerStats.overrunCount/innerStats.sampleCount)*100);
}

void cmdTasks(const String &args) 
{
    char taskListBuffer[512];
    vTaskList(taskListBuffer);
    LOG("Task List:");
    LOG("%s", taskListBuffer);
}

void cmdSetMode(const String &args) 
{
    if (!globalController) {
        LOG_ERR("Controller not set!");
        return;
    }
    String argLower = args;
    argLower.toLowerCase();

    SystemCommand cmd;
    cmd.type = CMD_SET_MODE;

    if (argLower.indexOf("assist") >= 0) 
    {
        LOG_INF("Changing Flight Control mode to: ATTITUDE_MODE.");
        cmd.mode = ATTITUDE_MODE;
        CommandSystem::instance().pushCommand(cmd);
    } 
    else if (argLower.indexOf("stabilized") >= 0) 
    {
        LOG_INF("Changing Flight Control mode to: RATE_MODE.");
        cmd.mode = RATE_MODE;
        CommandSystem::instance().pushCommand(cmd);
    } 
    else 
    {
        LOG("Unknown mode. Use 'assist' or 'stabilized'.");
    }
}

void cmdCalibrateIMU(const String &args) 
{
    // Accept either no arguments or "imu" as the argument.
    String trimmed = args;
    trimmed.trim();
    trimmed.toLowerCase();

    if (trimmed.length() == 0 || trimmed.equals("imu")) {
        // Route through CommandSystem to ensure proper task pause/resume.
        // The CMD_CALIBRATE handler in CommandSystem pauses tasks, calibrates, 
        // then resumes — avoiding mutex contention with the running IMU task.
        SystemCommand cmd;
        cmd.type = CMD_CALIBRATE;
        CommandSystem::instance().pushCommand(cmd);
        
        LOG("IMU calibration queued. Tasks will pause during calibration.");
    } 
    else 
    {
        LOG("Unknown calibration target. Use 'calibrate imu'.");
    }
}

void cmdFlash(const String &args) 
{
    if (!globalFlashTelemetry) 
    {
        LOG("Flash telemetry not initialized!");
        return;
    }

    // Trim & lowercase for simple parsing
    String in = args;
    in.trim();
    in.toLowerCase();

    // Split first word (sub-command) from the rest (parameter)
    int spacePos = in.indexOf(' ');
    String cmd = (spacePos < 0) ? in : in.substring(0, spacePos);
    String param = (spacePos < 0) ? "" : in.substring(spacePos + 1);
    param.trim();

    if (cmd == "start") 
    {
        globalFlashTelemetry->startLogging();
        LOG("Flash logging STARTED");
    }
    else if (cmd == "stop") 
    {
        globalFlashTelemetry->stopLogging();
        LOG("Flash logging STOPPED");
    }
    else if (cmd == "list") 
    {
        LOG("Listing flash logs:");
        globalFlashTelemetry->listLogs();
    }
    else if (cmd == "dump") 
    {
        if (param.length() == 0) 
        {
            LOG("Usage: flash dump <index>");
        } 
        else 
        {
            int idx = param.toInt();
            LOG("Dumping log %d:\n", idx);
            globalFlashTelemetry->dumpLog(idx);
        }
    }
    else if (cmd == "delete" || cmd == "del" || cmd == "rm") 
    {
        if (param.length() == 0) 
        {
            LOG("Usage: flash delete <index>");
        } 
        else 
        {
            int idx = param.toInt();
            LOG("Deleting log %d: ", idx);
            globalFlashTelemetry->deleteLog(idx);
        }
    }
    else if (cmd == "reset") 
    {
        LOG("Formatting LittleFS (erasing all logs)...");
        globalFlashTelemetry->reset();
        LOG("Done.");
    }
    else 
    {
        LOG("Unknown flash command. Available:");
        LOG("  flash start       → begin a new flight log");
        LOG("  flash stop        → end current flight log");
        LOG("  flash list        → list existing logs");
        LOG("  flash dump <idx>  → stream log #<idx> over serial");
        LOG("  flash delete <idx>→ remove log #<idx>");
        LOG("  flash reset       → erase entire LittleFS");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Config Commands (using ConfigRegistry)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Glob-style pattern matching for config keys.
 * 
 * Supports:
 *   "*"         - matches everything
 *   "rate.*"    - matches rate.roll.kp, rate.pitch.ti, etc.
 *   "rate.*.kp" - matches rate.roll.kp, rate.pitch.kp, etc.
 *   "rate.roll.*" - matches rate.roll.kp, rate.roll.ti, etc.
 */
static bool matchGlob(const char* pattern, const char* key)
{
    while (*pattern && *key)
    {
        if (*pattern == '*')
        {
            pattern++;
            if (*pattern == '\0') return true;  // trailing * matches rest
            
            // Try matching rest of pattern at each position
            while (*key)
            {
                if (matchGlob(pattern, key)) return true;
                key++;
            }
            return matchGlob(pattern, key);  // Check if pattern also exhausted
        }
        else if (*pattern == *key)
        {
            pattern++;
            key++;
        }
        else
        {
            return false;
        }
    }
    
    // Handle trailing wildcards
    while (*pattern == '*') pattern++;
    
    return (*pattern == '\0' && *key == '\0');
}

void cmdConfig(const String &args)
{
    auto& reg = ConfigRegistry::instance();
    
    // Parse subcommand and arguments
    int spaceIdx = args.indexOf(' ');
    String cmd = (spaceIdx > 0) ? args.substring(0, spaceIdx) : args;
    String remainder = (spaceIdx > 0) ? args.substring(spaceIdx + 1) : "";
    cmd.trim();
    remainder.trim();
    
    if (cmd == "list" || cmd.isEmpty())
    {
        // List all params or params matching a pattern
        String pattern = remainder.isEmpty() ? "*" : remainder;
        auto params = reg.getAllParams();
        
        // Collect matching keys and sort alphabetically
        std::vector<std::string> matchingKeys;
        for (const auto& kv : params)
        {
            if (matchGlob(pattern.c_str(), kv.first.c_str()))
            {
                matchingKeys.push_back(kv.first);
            }
        }
        std::sort(matchingKeys.begin(), matchingKeys.end());
        
        // Display sorted params
        for (const auto& key : matchingKeys)
        {
            const auto& param = params[key];
            switch (param.type)
            {
                case ConfigType::FLOAT:
                    LOG("  %s = %.4f", key.c_str(), param.currentVal.f);
                    break;
                case ConfigType::INT32:
                    LOG("  %s = %d", key.c_str(), param.currentVal.i);
                    break;
                case ConfigType::UINT8:
                    LOG("  %s = %u", key.c_str(), param.currentVal.u8);
                    break;
                case ConfigType::BOOL:
                    LOG("  %s = %s", key.c_str(), param.currentVal.b ? "true" : "false");
                    break;
                case ConfigType::STRING:
                    LOG("  %s = \"%s\"", key.c_str(), param.currentVal.s);
                    break;
            }
        }
        LOG("(%d parameter%s)", matchingKeys.size(), matchingKeys.size() == 1 ? "" : "s");
    }
    else if (cmd == "get")
    {
        if (remainder.isEmpty())
        {
            LOG_ERR("Usage: config get <key|pattern>");
            return;
        }
        
        // Check if it's a pattern (contains *)
        bool isPattern = remainder.indexOf('*') >= 0;
        
        if (!isPattern)
        {
            // Exact key lookup
            auto optParam = reg.getParam(remainder.c_str());
            if (!optParam)
            {
                LOG_ERR("Unknown key: %s", remainder.c_str());
                return;
            }
            
            const auto& p = *optParam;
            switch (p.type)
            {
                case ConfigType::FLOAT:
                    LOG("%s = %.4f (default: %.4f, range: %.4f - %.4f)", 
                        p.key, p.currentVal.f, p.defaultVal.f, p.minVal.f, p.maxVal.f);
                    break;
                case ConfigType::INT32:
                    LOG("%s = %d (default: %d, range: %d - %d)", 
                        p.key, p.currentVal.i, p.defaultVal.i, p.minVal.i, p.maxVal.i);
                    break;
                case ConfigType::UINT8:
                    LOG("%s = %u (default: %u, range: %u - %u)", 
                        p.key, p.currentVal.u8, p.defaultVal.u8, p.minVal.u8, p.maxVal.u8);
                    break;
                case ConfigType::BOOL:
                    LOG("%s = %s (default: %s)", 
                        p.key, p.currentVal.b ? "true" : "false", p.defaultVal.b ? "true" : "false");
                    break;
                case ConfigType::STRING:
                    LOG("%s = \"%s\" (default: \"%s\")", 
                        p.key, p.currentVal.s, p.defaultVal.s);
                    break;
            }
            LOG("  Description: %s", p.description);
        }
        else
        {
            // Pattern matching - show all matching params with details
            auto params = reg.getAllParams();
            
            // Collect and sort matching keys
            std::vector<std::string> matchingKeys;
            for (const auto& kv : params)
            {
                if (matchGlob(remainder.c_str(), kv.first.c_str()))
                {
                    matchingKeys.push_back(kv.first);
                }
            }
            std::sort(matchingKeys.begin(), matchingKeys.end());
            
            if (matchingKeys.empty())
            {
                LOG_ERR("No keys matching: %s", remainder.c_str());
                return;
            }
            
            for (const auto& key : matchingKeys)
            {
                const auto& p = params[key];
                switch (p.type)
                {
                    case ConfigType::FLOAT:
                        LOG("%s = %.4f (range: %.4f - %.4f)", 
                            key.c_str(), p.currentVal.f, p.minVal.f, p.maxVal.f);
                        break;
                    case ConfigType::INT32:
                        LOG("%s = %d (range: %d - %d)", 
                            key.c_str(), p.currentVal.i, p.minVal.i, p.maxVal.i);
                        break;
                    case ConfigType::UINT8:
                        LOG("%s = %u (range: %u - %u)", 
                            key.c_str(), p.currentVal.u8, p.minVal.u8, p.maxVal.u8);
                        break;
                    case ConfigType::BOOL:
                        LOG("%s = %s", key.c_str(), p.currentVal.b ? "true" : "false");
                        break;
                    case ConfigType::STRING:
                        LOG("%s = \"%s\"", key.c_str(), p.currentVal.s);
                        break;
                }
            }
            LOG("(%d parameter%s)", matchingKeys.size(), matchingKeys.size() == 1 ? "" : "s");
        }
    }
    else if (cmd == "set")
    {
        // Parse: key value
        int valIdx = remainder.indexOf(' ');
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
        
        const auto& p = *optParam;
        bool success = false;
        
        switch (p.type)
        {
            case ConfigType::FLOAT:
                success = reg.set<float>(key.c_str(), valStr.toFloat());
                break;
            case ConfigType::INT32:
                success = reg.set<int32_t>(key.c_str(), valStr.toInt());
                break;
            case ConfigType::UINT8:
                success = reg.set<uint8_t>(key.c_str(), (uint8_t)valStr.toInt());
                break;
            case ConfigType::BOOL:
            {
                valStr.toLowerCase();
                bool val = (valStr == "true" || valStr == "1" || valStr == "yes");
                success = reg.set<bool>(key.c_str(), val);
                break;
            }
            case ConfigType::STRING:
                success = reg.set<String>(key.c_str(), valStr);
                break;
        }
        
        if (success)
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
        ConfigPersistence::load();
        LOG("Loaded configuration from NVS");
    }
    else if (cmd == "defaults")
    {
        // Re-init registry to reset all params to schema defaults
        reg.init();
        LOG("Reset all parameters to defaults (not saved yet)");
    }
    else
    {
        LOG("Unknown config command. Available:");
        LOG("  config list [pattern] → list params (e.g., 'rate.*')");
        LOG("  config get <key>      → show param details");
        LOG("  config set <key> <val>→ set param value");
        LOG("  config save           → save dirty params to NVS");
        LOG("  config load           → reload from NVS");
        LOG("  config defaults       → reset to factory defaults");
    }
}

// Define the command table.
CLICommand cliCommands[] = {
    { "help","      Show help message",                                                        cmdHelp         },
    { "reset","     Resets the Flight Controller",                                             cmdReset        },
    { "stats","     Show control loop statistics",                                             cmdStats        },
    { "tasks","     Show FreeRTOS task stats",                                                 cmdTasks        },
    { "setmode","   Set mode; usage: setmode assist|stabilized",                               cmdSetMode      },
    { "calibrate"," Calibrate the IMU; usage: calibrate imu",                                  cmdCalibrateIMU },
    { "flash","     Flash functionalities; usage: flash list|start|stop|dump|rm|reset",        cmdFlash        },
    { "config","    Config commands; usage: config list|get|set|save|load|defaults",           cmdConfig       }
};

const size_t numCLICommands = sizeof(cliCommands) / sizeof(cliCommands[0]);
