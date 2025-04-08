#include "src/cli/CLICommands.h"
#include "src/controller/ArduFliteController.h"  // So we can access controller functions, if needed.
#include <FreeRTOS.h>
#include <task.h>

// Forward declarations of command functions.
void cmdHelp(const String &args);
void cmdStats(const String &args);
void cmdTasks(const String &args);
void cmdSetMode(const String &args);

// Global pointer for CLI command functions to access the controller,
// this can be set from the CLI task or a CLI initialization function.
static ArduFliteController* globalController = nullptr;

/**
 * @brief Allows the CLI task to set the controller pointer.
 */
void setCLIController(ArduFliteController* controller) {
    globalController = controller;
}

// Define the command table.
CLICommand cliCommands[] = {
    { "help",    "Show help message",                           cmdHelp     },
    { "stats",   "Show control loop statistics",                cmdStats    },
    { "tasks",   "Show FreeRTOS task stats",                    cmdTasks    },
    { "setmode", "Set mode; usage: setmode assist|stabilized",  cmdSetMode  }
};

const size_t numCLICommands = sizeof(cliCommands) / sizeof(cliCommands[0]);

// Implementation of command functions:
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
    // Retrieve outer and inner loop stats.
    LoopStats outer = globalController->getOuterLoopStats();
    LoopStats inner = globalController->getInnerLoopStats();
    Serial.printf("Outer Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, samples: %lu\n",
                  outer.avgDt, outer.maxDt, outer.overrunCount, outer.sampleCount);
    Serial.printf("Inner Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, samples: %lu\n",
                  inner.avgDt, inner.maxDt, inner.overrunCount, inner.sampleCount);
}

void cmdTasks(const String &args) {
    // Use FreeRTOS vTaskList to generate a table.
    // Make sure configUSE_TRACE_FACILITY is set to 1 in FreeRTOSConfig.h.
    char taskList[512];
    vTaskList(taskList);
    Serial.println("Task List:");
    Serial.println(taskList);
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
