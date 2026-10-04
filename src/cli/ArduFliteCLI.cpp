/**
 * ArduFliteCLI.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include <chrono>

#include "src/cli/ArduFliteCLI.h"

#include "src/hal/board/Board.h"
#include "src/cli/CLICommandContext.h"
#include "src/cli/CLICommandUtils.h"
#include "src/cli/CLICommands.h"
#include "src/utils/Logging.h"

ArduFliteCLI::ArduFliteCLI(ArduFliteController* controller, arduflite::estimation::InertialSubsystem* imu, ArduFliteFlashTelemetry* flashTelemetry)
    : controller(controller), imu(imu), flashTelemetry(flashTelemetry)
{
    // Set the global pointers for CLI commands.
    setCliController(controller);
    setCliIMU(imu);
    setFlashTelemetry(flashTelemetry);
}

void ArduFliteCLI::startTask() {
    if (_scheduler == nullptr) {
        LOG_ERR("CLI startTask() before setScheduler() - CLI will not run.");
        return;
    }

    // Priority::Cli is 1, NOT 0. AGENTS.md's prose says 0; the ladder in
    // Scheduler.h is authoritative, and dropping the CLI a level below the
    // other background tasks would starve it behind telemetry.
    const arduflite::hal::TaskConfig cfg{
        "CLITask", kStackBytes, arduflite::hal::Priority::Cli, -1
    };

    auto task = _scheduler->spawn(cfg, cliTask, this);
    if (!task) {
        LOG_ERR("CLI task creation failed: %s", arduflite::toString(task.status()));
    }
}

void ArduFliteCLI::cliTask(void* parameters) {
    (void)parameters;
    String inputLine = "";

    LOG("CLI Task started. Type 'help' for available commands.");

    while (true) {
        // Read input from Serial.
        auto& console = arduflite::board::Board::instance().console();
        while (console.available() > 0) {
            const int byte = console.readByte();
            if (byte < 0) { break; }
            char c = static_cast<char>(byte);
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
        arduflite::board::Board::instance().scheduler().sleepFor(
            std::chrono::milliseconds{ 10 });
    }
}
