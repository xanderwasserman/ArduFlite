#include "src/cli/ArduFliteCLI.h"
#include "src/cli/CLICommands.h"
#include "src/cli/CLICommandsConfig.h"

ArduFliteCLI::ArduFliteCLI(ArduFliteController* controller)
    : controller(controller)
{
    // Set the global controller pointer for CLI commands.
    setCLIController(controller);
}

void ArduFliteCLI::startTask() {
    // Create the CLI task.
    xTaskCreate(cliTask, "CLI Task", 4096, this, 1, nullptr);
}

void ArduFliteCLI::cliTask(void* parameters) {
    ArduFliteCLI* cli = static_cast<ArduFliteCLI*>(parameters);
    String inputLine = "";
    
    Serial.println("CLI Task started. Type 'help' for available commands.");
    
    while (true) {
        // Read input from Serial.
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                // Process the command line if non-empty.
                if (inputLine.length() > 0) {
                    inputLine.trim();
                    // Extract the first token (the command) and the remainder (arguments).
                    int spaceIdx = inputLine.indexOf(' ');
                    String cmd, args;
                    if (spaceIdx == -1) {
                        cmd = inputLine;
                        args = "";
                    } else {
                        cmd = inputLine.substring(0, spaceIdx);
                        args = inputLine.substring(spaceIdx + 1);
                    }
                    bool found = false;
                    // Iterate over the registered commands.
                    for (size_t i = 0; i < numCLICommands; i++) {
                        if (cmd.equalsIgnoreCase(cliCommands[i].command)) {
                            // Command found; execute its function.
                            cliCommands[i].execute(args);
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        Serial.println("Unknown command. Type 'help' for available commands.");
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
