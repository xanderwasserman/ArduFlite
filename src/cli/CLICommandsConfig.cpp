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

// Global pointer for the controller (accessible in command functions).
ArduFliteController* globalController = nullptr;
ArduFliteIMU* globalIMU = nullptr;
ArduFliteFlashTelemetry* globalFlashTelemetry = nullptr;

void setCliController(ArduFliteController* controller) {
    globalController = controller;
}

void setCliIMU(ArduFliteIMU* imu) {
    globalIMU = imu;
}

void setFlashTelemetry(ArduFliteFlashTelemetry* flashTelemetry) {
    globalFlashTelemetry = flashTelemetry;
}

// Command functions:
void cmdHelp(const String &args) {
    LOG("Available commands:");
    for (size_t i = 0; i < numCLICommands; i++) {
        LOG_N("  ");
        LOG_N("%s", cliCommands[i].command);
        LOG_N(" - ");
        LOG("%s", cliCommands[i].description);
    }
}

void cmdReset(const String &args) {
    LOG("Resetting system...");
    ESP.restart();
}

void cmdStats(const String &args) {
    if (!globalController) {
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

void cmdTasks(const String &args) {
    char taskListBuffer[512];
    vTaskList(taskListBuffer);
    LOG("Task List:");
    LOG("%s", taskListBuffer);
}

void cmdSetMode(const String &args) {
    if (!globalController) {
        LOG_ERR("Controller not set!");
        return;
    }
    String argLower = args;
    argLower.toLowerCase();
    if (argLower.indexOf("assist") >= 0) {
        globalController->setMode(ASSIST_MODE);
        LOG("Mode set to ASSIST_MODE.");
    } else if (argLower.indexOf("stabilized") >= 0) {
        globalController->setMode(STABILIZED_MODE);
        LOG("Mode set to STABILIZED_MODE.");
    } else {
        LOG("Unknown mode. Use 'assist' or 'stabilized'.");
    }
}

void cmdCalibrateIMU(const String &args) {
    if (!globalController || !globalIMU) {
        LOG("CLI Controller or IMU reference not set!");
        return;
    }
    // Accept either no arguments or "imu" as the argument.
    String trimmed = args;
    trimmed.trim();
    trimmed.toLowerCase();
    if (trimmed.length() == 0 || trimmed.equals("imu")) {
        LOG("Starting IMU calibration...");
        bool success = globalIMU->selfCalibrate();
        if (success) {
            LOG("IMU calibration complete.");
        } else {
            LOG("IMU calibration failed.");
        }
    } else {
        LOG("Unknown calibration target. Use 'calibrate imu'.");
    }
}

void cmdFlash(const String &args) {
    if (!globalFlashTelemetry) {
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

    if (cmd == "start") {
        globalFlashTelemetry->startLogging();
        LOG("Flash logging STARTED");
    }
    else if (cmd == "stop") {
        globalFlashTelemetry->stopLogging();
        LOG("Flash logging STOPPED");
    }
    else if (cmd == "list") {
        LOG("Listing flash logs:");
        globalFlashTelemetry->listLogs();
    }
    else if (cmd == "dump") {
        if (param.length() == 0) {
            LOG("Usage: flash dump <index>");
        } else {
            int idx = param.toInt();
            LOG("Dumping log %d:\n", idx);
            globalFlashTelemetry->dumpLog(idx);
        }
    }
    else if (cmd == "delete" || cmd == "del" || cmd == "rm") {
        if (param.length() == 0) {
            LOG("Usage: flash delete <index>");
        } else {
            int idx = param.toInt();
            LOG("Deleting log %d: ", idx);
            globalFlashTelemetry->deleteLog(idx);
        }
    }
    else if (cmd == "reset") {
        LOG("Formatting LittleFS (erasing all logs)...");
        globalFlashTelemetry->reset();
        LOG("Done.");
    }
    else {
        LOG("Unknown flash command. Available:");
        LOG("  flash start       → begin a new flight log");
        LOG("  flash stop        → end current flight log");
        LOG("  flash list        → list existing logs");
        LOG("  flash dump <idx>  → stream log #<idx> over serial");
        LOG("  flash delete <idx>→ remove log #<idx>");
        LOG("  flash reset       → erase entire LittleFS");
    }
}

// Define the command table.
CLICommand cliCommands[] = {
    { "help","      Show help message",                                                        cmdHelp         },
    { "reset","     Resets the Flight Controller",                                             cmdReset         },
    { "stats","     Show control loop statistics",                                             cmdStats        },
    { "tasks","     Show FreeRTOS task stats",                                                 cmdTasks        },
    { "setmode","   Set mode; usage: setmode assist|stabilized",                               cmdSetMode      },
    { "calibrate"," Calibrate the IMU; usage: calibrate imu",                                  cmdCalibrateIMU },
    { "flash","     Flash functionalities; usage: flash list|start|stop|dump|rm|reset",        cmdFlash }
};

const size_t numCLICommands = sizeof(cliCommands) / sizeof(cliCommands[0]);
