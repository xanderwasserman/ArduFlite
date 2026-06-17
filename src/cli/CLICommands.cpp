/**
 * CLICommands.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/CLICommands.h"
#include "src/utils/Logging.h"

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

const CLICommand cliCommands[] = {
    { "help","      Show help message",                                                        cmdHelp         },
    { "reset","     Resets the Flight Controller",                                             cmdReset        },
    { "stats","     Show control loop statistics",                                             cmdStats        },
    { "tasks","     Show FreeRTOS task stats",                                                 cmdTasks        },
    { "setmode","   Set mode; usage: setmode assist|stabilized",                               cmdSetMode      },
    { "calibrate"," Calibrate the IMU; usage: calibrate imu",                                  cmdCalibrateIMU },
    { "flash","     Flash functionalities; usage: flash list|start|stop|dump|rm|reset",        cmdFlash        },
    { "config","    Config commands; usage: config list|get|set|save|load|defaults",           cmdConfig       },
    { "stream","    Stream live telemetry; usage: stream [freq_hz]",                           cmdStream       },
    { "test","      Run integration test; usage: test loops",                                  cmdTest         }
};

const size_t numCLICommands = sizeof(cliCommands) / sizeof(cliCommands[0]);
