/**
 * CommandSystem.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 16 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "CommandSystem.h"
#include "src/utils/Logging.h"

CommandSystem::CommandSystem() 
{
    // Create a queue that can hold 10 SystemCommand items.
    commandQueue = xQueueCreate(10, sizeof(SystemCommand));
    if (!commandQueue) {
        LOG_ERR("Failed to create CommandSystem queue!");
    }
}

CommandSystem::~CommandSystem() 
{
    if (commandQueue != nullptr) 
    {
        vQueueDelete(commandQueue);
    }
}

bool CommandSystem::pushCommand(const SystemCommand &cmd) 
{
    // Attempt to push the command; wait up to 10ms if necessary.
    return (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS);
}

void CommandSystem::processCommands(ArduFliteController *controller, ArduFliteIMU *imu) 
{
    SystemCommand cmd;
    // Process all pending commands (non-blocking)
    while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) 
    {
        switch (cmd.type) 
        {
            case CMD_RESET:
                LOG_INF("Processing RESET command...");
                // Here we might want to reset the system.
                ESP.restart();
                break;

            case CMD_CALIBRATE:
                LOG_INF("Processing CALIBRATE command...");
                if (imu != nullptr) {
                    imu->selfCalibrate();
                } 
                else 
                {
                    LOG_ERR("IMU pointer not provided.");
                }
                break;

            case CMD_MODE_ASSIST:
                LOG_INF("Processing MODE ASSIST command...");
                if (controller != nullptr) 
                {
                    controller->setMode(ASSIST_MODE);
                } 
                else 
                {
                    LOG_ERR("Controller pointer not provided.");
                }
                break;

            case CMD_MODE_STABILIZED:
                LOG_INF("Processing MODE STABILIZED command...");
                if (controller != nullptr) 
                {
                    controller->setMode(STABILIZED_MODE);
                } 
                else 
                {
                    LOG_ERR("Controller pointer not provided.");
                }
                break;

            default:
                LOG_WARN("Received unknown command type.");
                break;
        }
    }
}
