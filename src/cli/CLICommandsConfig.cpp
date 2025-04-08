#include "CLICommandsConfig.h"
#include <FreeRTOS.h>
#include <task.h>

// Global pointer for the controller (accessible in command functions).
ArduFliteController* globalController = nullptr;

void setCLIController(ArduFliteController* controller) {
    globalController = controller;
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

// Define the command table.
CLICommand cliCommands[] = {
    { "help",    "Show help message",                         cmdHelp },
    { "stats",   "Show control loop statistics",              cmdStats },
    { "tasks",   "Show FreeRTOS task stats",                  cmdTasks },
    { "setmode", "Set mode; usage: setmode assist|stabilized",  cmdSetMode }
};

const size_t numCLICommands = sizeof(cliCommands) / sizeof(cliCommands[0]);
