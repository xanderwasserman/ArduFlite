/**
 * CommandSystem.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 16 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/CommandSystem.h"
#include "src/mission_planner/MissionPlanner.h"
#include "src/utils/Logging.h"
#include "src/utils/ConfigHelpers.h"
#include "src/utils/ControlMixer.h"
#include "include/ConfigKeys.h"
#include "include/AircraftConfiguration.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"

extern MissionPlanner           mission;
extern ArduFliteFlashTelemetry  flashTelemetry;

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
        LOG_ERR("FATAL: Failed to create CommandSystem queue — system will restart.");
        ESP.restart();
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
    if (!commandQueue_)
    {
        LOG_ERR("CommandSystem: no queue — command dropped!");
        return false;
    }
    // Never block producers such as RC callbacks; stale setpoints are worse than
    // dropped setpoints under queue pressure.
    if (xQueueSend(commandQueue_, &cmd, 0) != pdPASS)
    {
        static unsigned long lastWarnMs = 0;
        unsigned long nowMs = millis();
        if (nowMs - lastWarnMs > 1000)
        {
            LOG_WARN("CommandSystem: queue full — command dropped");
            lastWarnMs = nowMs;
        }
        return false;
    }
    return true;
}

void CommandSystem::processCommands(ArduFliteController* controller, ArduFliteIMU* imu,
                                     ArdufliteCRSFReceiver* receiver, ArdufliteCRSFTelemetry* crsfTelemetry)
{
    if (!commandQueue_) return;
    SystemCommand cmd;

    // Process up to MAX_COMMANDS_PER_TICK pending commands (non-blocking)
    // This prevents queue backup during rapid command sequences
    static constexpr int MAX_COMMANDS_PER_TICK = 10;
    int processed = 0;

    while (processed < MAX_COMMANDS_PER_TICK &&
           xQueueReceive(commandQueue_, &cmd, 0) == pdTRUE)
    {
        processed++;

        switch (cmd.type)
        {
            case CMD_RESET:
            {
                LOG_DBG("Processing RESET command...");
                ESP.restart();
                break;
            }

            case CMD_CALIBRATE:
            {
                LOG_DBG("Processing CALIBRATE command...");
                if (imu != nullptr && controller != nullptr && crsfTelemetry != nullptr)
                {
                    // Pause controller tasks during calibration to avoid servo glitches
                    controller->pauseTasks();

                    // Pause CRSF telemetry to prevent WDT timeout during long calibration
                    crsfTelemetry->pauseTask();

                    // selfCalibrate() internally handles IMU task pause/resume
                    // and resets filter state after new offsets are applied
                    if (!imu->selfCalibrate())
                    {
                        LOG_ERR("IMU calibration failed!");
                    }

                    // Resume CRSF telemetry
                    crsfTelemetry->resumeTask();

                    // Resume controller tasks
                    controller->resumeTasks();
                }
                else
                {
                    LOG_ERR("CMD_CALIBRATE: requires non-null IMU, Controller, and CRSF telemetry pointers — one is missing.");
                }
                break;
            }

            case CMD_SET_MODE:
            {
                LOG_DBG("Processing CMD_SET_MODE: mode = %d", cmd.mode);
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

            case CMD_SET_SETPOINT:
            {
                LOG_DBG("Processing CMD_SET_SETPOINT: roll=%.3f, pitch=%.3f, yaw=%.3f",
                        cmd.setpoint.roll, cmd.setpoint.pitch, cmd.setpoint.yaw);

                if (controller != nullptr)
                {
                    if (controller->getMode() == ATTITUDE_MODE)
                    {
                        controller->setAttitudeSetpoint(cmd.setpoint);
                    }
                    else
                    {
                        controller->setRateSetpoint(cmd.setpoint);
                    }
                }
                else
                {
                    LOG_ERR("CMD_SET_SETPOINT: Controller pointer not provided.");
                }
                break;
            }

            case CMD_SET_MISSION:
            {
                LOG_DBG("Processing CMD_SET_MISSION: state=%s", cmd.x_value ? "START" : "STOP");

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
                LOG_DBG("Processing CMD_SET_ARM: state=%s", cmd.x_value ? "YES" : "NO");
                if (controller != nullptr)
                {
                    if (cmd.x_value)
                    {
                        // Run preflight checks and arm if passed
                        bool armed = controller->arm(receiver);
                        if (!armed)
                        {
                            LOG_ERR("ARM REJECTED - preflight checks failed!");
                        }
                        else
                        {
#if AIRCRAFT_TYPE == AIRCRAFT_TYPE_POWERED
                            // Start logging only when both armed AND throttle cut is off.
                            // If throttle cut is still active, logging is deferred until
                            // the pilot releases it (handled in CMD_SET_THROTTLE_CUT).
                            if (!flashTelemetry.isLogging() && !controller->isThrottleCut())
                            {
                                if (!flashTelemetry.startLogging())
                                {
                                    LOG_ERR("Flight log FAILED to start on arm — flight will NOT be recorded!");
                                }
                            }
                            else if (flashTelemetry.isLogging())
                            {
                                LOG_INF("Flash log already active — not restarting on re-arm.");
                            }
                            else
                            {
                                LOG_INF("Throttle cut active — logging deferred until throttle cut is released.");
                            }
#else
                            // AIRCRAFT_TYPE_GLIDER: no logging action on arm — logging starts
                            // on INFLIGHT transition in StateManagement.cpp instead.
#endif
                        }
                    }
                    else
                    {
                        controller->disarm();
                        if (flashTelemetry.isLogging() &&
                            (imu == nullptr || imu->getFlightState() != INFLIGHT))
                        {
                            flashTelemetry.stopLogging();
                        }
                    }
                }
                else
                {
                    LOG_ERR("CMD_SET_ARM: Controller pointer not provided.");
                }
                break;
            }

            case CMD_SET_THROTTLE_CUT:
            {
                LOG_DBG("Processing CMD_SET_THROTTLE_CUT: state=%s", cmd.x_value ? "YES" : "NO");
                if (controller != nullptr)
                {
                    controller->cutThrottle(cmd.x_value);
#if AIRCRAFT_TYPE == AIRCRAFT_TYPE_POWERED
                    // Throttle cut released while armed — begin logging now.
                    // Covers the case where the pilot arms first, then enables the motor.
                    if (!cmd.x_value && controller->isArmed() && !flashTelemetry.isLogging())
                    {
                        if (!flashTelemetry.startLogging())
                        {
                            LOG_ERR("Flight log FAILED to start on throttle-cut release — flight will NOT be recorded!");
                        }
                    }
#endif
                }
                else
                {
                    LOG_ERR("CMD_SET_THROTTLE_CUT: Controller pointer not provided.");
                }
                break;
            }

            case CMD_SET_SETPOINT_ROLL:
            {
                LOG_DBG("Processing CMD_SET_SETPOINT_ROLL: roll=%.3f", cmd.setpoint.roll);
                if (controller != nullptr)
                {
                    controller->setAttitudeSetpointAxis(0, cmd.setpoint.roll);
                }
                else
                {
                    LOG_ERR("CMD_SET_SETPOINT_ROLL: Controller pointer not provided.");
                }
                break;
            }

            case CMD_SET_SETPOINT_PITCH:
            {
                LOG_DBG("Processing CMD_SET_SETPOINT_PITCH: pitch=%.3f", cmd.setpoint.pitch);
                if (controller != nullptr)
                {
                    controller->setAttitudeSetpointAxis(1, cmd.setpoint.pitch);
                }
                else
                {
                    LOG_ERR("CMD_SET_SETPOINT_PITCH: Controller pointer not provided.");
                }
                break;
            }

            case CMD_SET_SETPOINT_YAW:
            {
                LOG_DBG("Processing CMD_SET_SETPOINT_YAW: yaw=%.3f", cmd.setpoint.yaw);
                if (controller != nullptr)
                {
                    controller->setAttitudeSetpointAxis(2, cmd.setpoint.yaw);
                }
                else
                {
                    LOG_ERR("CMD_SET_SETPOINT_YAW: Controller pointer not provided.");
                }
                break;
            }

            case CMD_SET_SETPOINT_THROTTLE:
            {
                LOG_DBG("Processing CMD_SET_SETPOINT_THROTTLE: throttle=%.3f", cmd.value);
                if (controller != nullptr)
                {
                    controller->setThrottleSetpoint(cmd.value);
                }
                else
                {
                    LOG_ERR("CMD_SET_SETPOINT_THROTTLE: Controller pointer not provided.");
                }
                break;
            }

            // ─────────────────────────────────────────────────────────────────
            // Config Update Commands (pushed by observers)
            // ─────────────────────────────────────────────────────────────────
            case CMD_UPDATE_RATE_ROLL_PID:
            {
                LOG_DBG("Processing CMD_UPDATE_RATE_ROLL_PID");
                if (controller != nullptr)
                {
                    PIDConfig cfg = ConfigHelpers::buildPIDConfig(CONFIG_KEY_RATE_ROLL_PREFIX);
                    controller->setRatePIDConfig(RATE_ROLL_LOOP, cfg);
                }
                break;
            }

            case CMD_UPDATE_RATE_PITCH_PID:
            {
                LOG_DBG("Processing CMD_UPDATE_RATE_PITCH_PID");
                if (controller != nullptr)
                {
                    PIDConfig cfg = ConfigHelpers::buildPIDConfig(CONFIG_KEY_RATE_PITCH_PREFIX);
                    controller->setRatePIDConfig(RATE_PITCH_LOOP, cfg);
                }
                break;
            }

            case CMD_UPDATE_RATE_YAW_PID:
            {
                LOG_DBG("Processing CMD_UPDATE_RATE_YAW_PID");
                if (controller != nullptr)
                {
                    PIDConfig cfg = ConfigHelpers::buildPIDConfig(CONFIG_KEY_RATE_YAW_PREFIX);
                    controller->setRatePIDConfig(RATE_YAW_LOOP, cfg);
                }
                break;
            }

            case CMD_UPDATE_RATE_OUT_ALPHA:
            {
                LOG_DBG("Processing CMD_UPDATE_RATE_OUT_ALPHA");
                if (controller != nullptr)
                {
                    float alpha = ConfigRegistry::instance().get<float>(CONFIG_KEY_RATE_OUT_LP_ALPHA);
                    controller->setRateOutputAlpha(alpha);
                }
                break;
            }

            case CMD_UPDATE_ATT_ROLL_PID:
            {
                LOG_DBG("Processing CMD_UPDATE_ATT_ROLL_PID");
                if (controller != nullptr)
                {
                    PIDConfig cfg = ConfigHelpers::buildPIDConfig(CONFIG_KEY_ATT_ROLL_PREFIX);
                    controller->setAttitudePIDConfig(ATTITUDE_ROLL_LOOP, cfg);
                }
                break;
            }

            case CMD_UPDATE_ATT_PITCH_PID:
            {
                LOG_DBG("Processing CMD_UPDATE_ATT_PITCH_PID");
                if (controller != nullptr)
                {
                    PIDConfig cfg = ConfigHelpers::buildPIDConfig(CONFIG_KEY_ATT_PITCH_PREFIX);
                    controller->setAttitudePIDConfig(ATTITUDE_PITCH_LOOP, cfg);
                }
                break;
            }

            case CMD_UPDATE_ATT_YAW_PID:
            {
                LOG_DBG("Processing CMD_UPDATE_ATT_YAW_PID");
                if (controller != nullptr)
                {
                    PIDConfig cfg = ConfigHelpers::buildPIDConfig(CONFIG_KEY_ATT_YAW_PREFIX);
                    controller->setAttitudePIDConfig(ATTITUDE_YAW_LOOP, cfg);
                }
                break;
            }

            case CMD_UPDATE_ATT_DEADBAND:
            {
                LOG_DBG("Processing CMD_UPDATE_ATT_DEADBAND");
                if (controller != nullptr)
                {
                    float deadband = ConfigRegistry::instance().get<float>(CONFIG_KEY_ATT_DEADBAND);
                    controller->setAttitudeDeadband(deadband);
                }
                break;
            }

            case CMD_UPDATE_MIXER:
            {
                LOG_DBG("Processing CMD_UPDATE_MIXER");
                ControlMixer::reloadConfig();
                break;
            }

            default:
            {
                LOG_WARN("Received unknown or unhandled command type: %d", cmd.type);
                break;
            }
        } // end switch
    } // end while
}
