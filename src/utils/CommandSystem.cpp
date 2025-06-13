/**
 * CommandSystem.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 16 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/CommandSystem.h"
#include "src/mission_planner/MissionPlanner.h"
#include "src/utils/Logging.h"

extern MissionPlanner mission;

CommandSystem& CommandSystem::instance() 
{
    static CommandSystem _inst;
    return _inst;
}

CommandSystem::CommandSystem() 
{
    // Create a queue that can hold 100 SystemCommand items.
    commandQueue_ = xQueueCreate(100, sizeof(SystemCommand));
    if (!commandQueue_) 
    {
        LOG_ERR("Failed to create CommandSystem queue!");
    }
}

CommandSystem::~CommandSystem() 
{
    if (commandQueue_) {
        vQueueDelete(commandQueue_);
    }
}

bool CommandSystem::pushCommand(const SystemCommand& cmd) 
{
    // wait up to 10 ms to enqueue
    if (!commandQueue_) return false;
    return xQueueSend(commandQueue_, &cmd, pdMS_TO_TICKS(10)) == pdPASS;
}

void CommandSystem::processCommands(ArduFliteController* controller, ArduFliteIMU* imu) 
{
    if (!commandQueue_) return;
    SystemCommand cmd;

   // Process exactly one pending command (non-blocking)
    if (xQueueReceive(commandQueue_, &cmd, 0) != pdTRUE) return;
    
    switch (cmd.type) 
    {
        case CMD_RESET:
        {
            LOG_INF("Processing RESET command...");
            ESP.restart();
            break;
        }

        case CMD_CALIBRATE:
        {
            LOG_INF("Processing CALIBRATE command...");
            if (imu != nullptr) 
            {
                imu->selfCalibrate();
            } 
            else 
            {
                LOG_ERR("IMU pointer not provided.");
            }
            break;
        }

        case CMD_SET_MODE:
        {
            LOG_INF("Processing CMD_SET_MODE: mode = %d", cmd.mode);
            if (controller != nullptr)
            {
                controller->setMode(cmd.mode);
            }
            else
            {
                LOG_ERR("CMD_SET_MODE: Controller pointer not provided.");
            }
            break;
        }

        case CMD_SET_CONFIG_PID:
        {
            // cmd.pidLoop is a ControlLoopType (e.g. ATTITUDE_ROLL_LOOP, RATE_YAW_LOOP, etc.)
            // cmd.pidConfig is a full PIDConfig struct
            LOG_INF("Processing CMD_SET_CONFIG_PID: loop = %d, kp=%.3f, ki=%.3f, kd=%.3f, outLimit=%.3f, maxI=%.3f, alpha=%.3f",
                    cmd.pidLoop,
                    cmd.pidConfig.kp,
                    cmd.pidConfig.ki,
                    cmd.pidConfig.kd,
                    cmd.pidConfig.outLimit,
                    cmd.pidConfig.maxIntegral,
                    cmd.pidConfig.derivativeAlpha);

            if (controller != nullptr)
            {
                LOG_ERR("Setting of PID values not supported yet! Please implement me :-)"); //TODO
            }
            else
            {
                LOG_ERR("CMD_SET_CONFIG_PID: Controller pointer not provided.");
            }
            break;
        }

        case CMD_SET_CONFIG_ATTITUDE:
        {
            
            
            // cmd.attitudeConfig is an EulerAngles { roll, pitch, yaw }
            // LOG_INF("Processing CMD_SET_CONFIG_ATTITUDE: roll=%.3f, pitch=%.3f, yaw=%.3f", cmd.attitudeConfig.roll, cmd.attitudeConfig.pitch, cmd.attitudeConfig.yaw);

            if (controller != nullptr)
            {
                if (controller->getMode() == ATTITUDE_MODE)
                {
                    controller->setAttitudeSetpoint(cmd.attitudeConfig);
                }
                else
                {
                    controller->setRateSetpoint(cmd.attitudeConfig);
                }
            }
            else
            {
                LOG_ERR("CMD_SET_CONFIG_ATTITUDE: Controller pointer not provided.");
            }
            break;
        }

        case CMD_SET_CONFIG_RATE_ALPHA:
        {
            // cmd.value is a single float
            LOG_INF("Processing CMD_SET_CONFIG_RATE_ALPHA: alpha=%.3f", cmd.value);
            if (controller != nullptr)
            {
                LOG_ERR("Setting of Rate controller Alpha is not supported yet! Please implement me :-)"); //TODO
            }
            else
            {
                LOG_ERR("CMD_SET_CONFIG_RATE_ALPHA: Controller pointer not provided.");
            }
            break;
        }

        case CMD_SET_MISSION:
        {
            // cmd.x_value is a bool
            LOG_INF("Processing CMD_SET_MISSION: state=%s", cmd.x_value?"START":"STOP");
            
            if (cmd.x_value && !mission.isRunning())
            {
                mission.start();
            }
            else if (!cmd.x_value && mission.isRunning())
            {
                mission.stop();
            }
            break;
        }

        case CMD_SET_ARM:
        {
            // cmd.x_value is a bool
            LOG_INF("Processing CMD_SET_ARM: state=%s", cmd.x_value?"YES":"NO");
            if (controller != nullptr)
            {
                if (cmd.x_value)
                {
                    controller->resumeTasks();
                }
                else
                {
                    controller->pauseTasks();
                }
                
            }
            else
            {
                LOG_ERR("CMD_SET_ARM: Controller pointer not provided.");
            }
            break;
        }

        case CMD_RECEIVER_SETPOINT_ROLL:
        {
            // cmd.attitudeConfig is an EulerAngles { roll, pitch, yaw }
            LOG_INF("Processing CMD_RECEIVER_SETPOINT_ROLL: roll=%.3f", cmd.attitudeConfig.roll);

            if (controller != nullptr)
            {
                controller->setAttitudeSetpoint_roll(cmd.attitudeConfig.roll);
            }
            else
            {
                LOG_ERR("CMD_RECEIVER_SETPOINT_ROLL: Controller pointer not provided.");
            }
            break;
        }
        case CMD_RECEIVER_SETPOINT_PITCH:
        {
            // cmd.attitudeConfig is an EulerAngles { roll, pitch, yaw }
            LOG_INF("Processing CMD_RECEIVER_SETPOINT_PITCH: pitch=%.3f", cmd.attitudeConfig.pitch);

            if (controller != nullptr)
            {
                controller->setAttitudeSetpoint_pitch(cmd.attitudeConfig.pitch);
            }
            else
            {
                LOG_ERR("CMD_RECEIVER_SETPOINT_PITCH: Controller pointer not provided.");
            }
            break;
        }
        case CMD_RECEIVER_SETPOINT_YAW:
        {
            // cmd.attitudeConfig is an EulerAngles { roll, pitch, yaw }
            LOG_INF("Processing CMD_RECEIVER_SETPOINT_YAW: yaw=%.3f", cmd.attitudeConfig.yaw);

            if (controller != nullptr)
            {
                controller->setAttitudeSetpoint_yaw(cmd.attitudeConfig.yaw);
            }
            else
            {
                LOG_ERR("CMD_RECEIVER_SETPOINT_YAW: Controller pointer not provided.");
            }
            break;
        }

        default:
        {
            LOG_WARN("Received unknown or unhandled command type: %d", cmd.type);
            break;
        }
    }
}