/**
 * CommandSystem.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 16 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "CommandSystem.h"

CommandSystem::CommandSystem() 
{
    // Create a queue that can hold 10 SystemCommand items.
    commandQueue = xQueueCreate(10, sizeof(SystemCommand));
    if (!commandQueue) {
    Serial.println("ERROR: Failed to create CommandSystem queue!");
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
                Serial.println("Processing RESET command...");
                // Here we might want to reset the system.
                ESP.restart();
                break;

            case CMD_CALIBRATE:
                Serial.println("Processing CALIBRATE command...");
                if (imu != nullptr) {
                    imu->selfCalibrate();
                } 
                else 
                {
                    Serial.println("IMU pointer not provided.");
                }
                break;

            case CMD_MODE_ASSIST:
                Serial.println("Processing MODE ASSIST command...");
                if (controller != nullptr) 
                {
                    controller->setMode(ASSIST_MODE);
                } 
                else 
                {
                    Serial.println("Controller pointer not provided.");
                }
                break;

            case CMD_MODE_STABILIZED:
                Serial.println("Processing MODE STABILIZED command...");
                if (controller != nullptr) 
                {
                    controller->setMode(STABILIZED_MODE);
                } 
                else 
                {
                    Serial.println("Controller pointer not provided.");
                }
                break;

            default:
                Serial.println("Received unknown command type.");
                break;
        }
    }
}
