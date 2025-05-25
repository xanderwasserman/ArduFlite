/**
 * CLICommandsConfig.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "CLICommandsConfig.h"

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
    Serial.println("Available commands:");
    for (size_t i = 0; i < numCLICommands; i++) {
        Serial.print("  ");
        Serial.print(cliCommands[i].command);
        Serial.print(" - ");
        Serial.println(cliCommands[i].description);
    }
}

void cmdStats(const String &args) {
    if (!globalController) {
        Serial.println("Controller not set!");
        return;
    }
    // Retrieve and print stats.
    LoopStats outerStats = globalController->getOuterLoopStats();
    LoopStats innerStats = globalController->getInnerLoopStats();
    Serial.printf("Outer Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, percentage: %lu\n",
                  outerStats.avgDt, outerStats.maxDt, outerStats.overrunCount, (outerStats.overrunCount/outerStats.sampleCount)*100);
    Serial.printf("Inner Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, percentage: %lu\n",
                  innerStats.avgDt, innerStats.maxDt, innerStats.overrunCount, (innerStats.overrunCount/innerStats.sampleCount)*100);
}

void cmdTasks(const String &args) {
    char taskListBuffer[512];
    vTaskList(taskListBuffer);
    Serial.println("Task List:");
    Serial.println(taskListBuffer);
}

void cmdSetMode(const String &args) {
    if (!globalController) {
        Serial.println("Controller not set!");
        return;
    }
    String argLower = args;
    argLower.toLowerCase();
    if (argLower.indexOf("assist") >= 0) {
        globalController->setMode(ASSIST_MODE);
        Serial.println("Mode set to ASSIST_MODE.");
    } else if (argLower.indexOf("stabilized") >= 0) {
        globalController->setMode(STABILIZED_MODE);
        Serial.println("Mode set to STABILIZED_MODE.");
    } else {
        Serial.println("Unknown mode. Use 'assist' or 'stabilized'.");
    }
}

void cmdCalibrateIMU(const String &args) {
    if (!globalController || !globalIMU) {
        Serial.println("CLI Controller or IMU reference not set!");
        return;
    }
    // Accept either no arguments or "imu" as the argument.
    String trimmed = args;
    trimmed.trim();
    trimmed.toLowerCase();
    if (trimmed.length() == 0 || trimmed.equals("imu")) {
        Serial.println("Starting IMU calibration...");
        bool success = globalIMU->selfCalibrate();
        if (success) {
            Serial.println("IMU calibration complete.");
        } else {
            Serial.println("IMU calibration failed.");
        }
    } else {
        Serial.println("Unknown calibration target. Use 'calibrate imu'.");
    }
}

void cmdFlash(const String &args) {
    if (!globalFlashTelemetry) {
        Serial.println("Flash telemetry not initialized!");
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
        Serial.println("→ Flash logging STARTED");
    }
    else if (cmd == "stop") {
        globalFlashTelemetry->stopLogging();
        Serial.println("→ Flash logging STOPPED");
    }
    else if (cmd == "list") {
        Serial.println("→ Listing flash logs:");
        globalFlashTelemetry->listLogs();
    }
    else if (cmd == "dump") {
        if (param.length() == 0) {
            Serial.println("Usage: flash dump <index>");
        } else {
            int idx = param.toInt();
            Serial.printf("→ Dumping log %d:\n", idx);
            globalFlashTelemetry->dumpLog(idx);
        }
    }
    else if (cmd == "delete" || cmd == "del" || cmd == "rm") {
        if (param.length() == 0) {
            Serial.println("Usage: flash delete <index>");
        } else {
            int idx = param.toInt();
            Serial.printf("→ Deleting log %d: ", idx);
            globalFlashTelemetry->deleteLog(idx);
        }
    }
    else if (cmd == "reset") {
        Serial.println("→ Formatting LittleFS (erasing all logs)...");
        globalFlashTelemetry->reset();
        Serial.println("→ Done.");
    }
    else {
        Serial.println("Unknown flash command. Available:");
        Serial.println("  flash start       → begin a new flight log");
        Serial.println("  flash stop        → end current flight log");
        Serial.println("  flash list        → list existing logs");
        Serial.println("  flash dump <idx>  → stream log #<idx> over serial");
        Serial.println("  flash delete <idx>→ remove log #<idx>");
        Serial.println("  flash reset       → erase entire LittleFS");
    }
}

// Define the command table.
CLICommand cliCommands[] = {
    { "help","      Show help message",                                                        cmdHelp         },
    { "stats","     Show control loop statistics",                                             cmdStats        },
    { "tasks","     Show FreeRTOS task stats",                                                 cmdTasks        },
    { "setmode","   Set mode; usage: setmode assist|stabilized",                               cmdSetMode      },
    { "calibrate"," Calibrate the IMU; usage: calibrate imu",                                  cmdCalibrateIMU },
    { "flash","     Flash functionalities; usage: flash list|start|stop|dump|rm|reset",        cmdFlash }
};

const size_t numCLICommands = sizeof(cliCommands) / sizeof(cliCommands[0]);
