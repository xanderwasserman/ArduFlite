/**
 * ArduFliteCLI.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/ArduFliteCLI.h"
#include "src/cli/CLICommandContext.h"
#include "src/cli/CLICommandUtils.h"
#include "src/cli/CLICommands.h"
#include "src/utils/Logging.h"

ArduFliteCLI::ArduFliteCLI(ArduFliteController* controller, ArduFliteIMU* imu, ArduFliteFlashTelemetry* flashTelemetry)
    : controller(controller), imu(imu), flashTelemetry(flashTelemetry)
{
    // Set the global pointers for CLI commands.
    setCliController(controller);
    setCliIMU(imu);
    setFlashTelemetry(flashTelemetry);
}

void ArduFliteCLI::startTask() {
    // Create the CLI task.
    xTaskCreate(cliTask, "CLI Task", 4096, this, 1, nullptr);
}

void ArduFliteCLI::cliTask(void* parameters) {
    (void)parameters;
    String inputLine = "";

    LOG("CLI Task started. Type 'help' for available commands.");

    while (true) {
        // Read input from Serial.
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                // Process the command line if non-empty.
                if (inputLine.length() > 0) {
                    ParsedCommand parsed = parseCommandArgs(inputLine);
                    bool found = false;
                    // Iterate over the registered commands.
                    for (size_t i = 0; i < numCLICommands; i++) {
                        if (parsed.command == cliCommands[i].command) {
                            // Command found; execute its function.
                            cliCommands[i].execute(parsed.remainder);
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        LOG("Unknown command. Type 'help' for available commands.");
                    }
                    inputLine = "";  // Clear the line.
                }
            } else {
                inputLine += c;
            }
        }
        // Short delay to yield.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
