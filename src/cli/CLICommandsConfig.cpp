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

void setCliController(ArduFliteController* controller) {
    globalController = controller;
}

void setCliIMU(ArduFliteIMU* imu) {
    globalIMU = imu;
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
    Serial.printf("Outer Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, samples: %lu\n",
                  outerStats.avgDt, outerStats.maxDt, outerStats.overrunCount, outerStats.sampleCount);
    Serial.printf("Inner Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, samples: %lu\n",
                  innerStats.avgDt, innerStats.maxDt, innerStats.overrunCount, innerStats.sampleCount);
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

// Define the command table.
CLICommand cliCommands[] = {
    { "help",       "Show help message",                            cmdHelp         },
    { "stats",      "Show control loop statistics",                 cmdStats        },
    { "tasks",      "Show FreeRTOS task stats",                     cmdTasks        },
    { "setmode",    "Set mode; usage: setmode assist|stabilized",   cmdSetMode      },
    { "calibrate",  "Calibrate the IMU; usage: calibrate imu",      cmdCalibrateIMU }
};

const size_t numCLICommands = sizeof(cliCommands) / sizeof(cliCommands[0]);
