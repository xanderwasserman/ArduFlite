/**
 * CommandSystem.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 16 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef COMMAND_SYSTEM_H
#define COMMAND_SYSTEM_H

#include <Arduino.h>
#include "src/controller/ArduFliteController.h"
#include "src/orientation/ArduFliteIMU.h"

/**
 * @brief Command types for system commands.
 */
enum SystemCommandType 
{
    CMD_NONE = 0,
    CMD_RESET,
    CMD_CALIBRATE,
    CMD_SET_MODE,
    CMD_SET_CONFIG_PID,
    CMD_SET_CONFIG_ATTITUDE,
    CMD_SET_CONFIG_RATE_ALPHA
};

/**
 * @brief Structure representing a system command.
 */
struct SystemCommand 
{
    SystemCommandType               type;

    ArduFliteMode                   mode;

    // for CMD_SET_PID_CONFIG
    ControlLoopType                 pidLoop;
    PIDConfig                       pidConfig;

    // for CMD_SET_ATTITUDE_CONFIG
    EulerAngles                     attitudeConfig;

    // for CMD_SET_RATE_ALPHA
    float                           rateAlpha;
};

/**
 * @brief CommandSystem class.
 *
 * This class implements a thread-safe command system using a FreeRTOS queue.
 * Any module can push a SystemCommand onto this queue.
 * The main loop (or another designated part of your code) should periodically call
 * processCommands() to handle and execute any pending commands, using the provided
 * pointers to ArduFliteController and ArduFliteIMU.
 */
class CommandSystem 
{
public:
    /**
     * @brief Constructor.
     *
     * Creates a FreeRTOS queue to hold SystemCommand items.
     */
    CommandSystem();

    /**
     * @brief Destructor.
     *
     * Frees the command queue.
     */
    ~CommandSystem();

    /**
     * @brief Pushes a command onto the queue.
     *
     * @param cmd The SystemCommand to push.
     * @return true if the command was successfully enqueued, false otherwise.
     */
    bool pushCommand(const SystemCommand &cmd);

    /**
     * @brief Processes all queued commands.
     *
     * This method dequeues pending commands (non-blocking) and executes them.
     * It accepts pointers to an ArduFliteController and ArduFliteIMU instance so that
     * command processing may invoke methods on these objects.
     *
     * @param controller Pointer to the ArduFliteController instance.
     * @param imu Pointer to the ArduFliteIMU instance.
     */
    void processCommands(ArduFliteController *controller, ArduFliteIMU *imu);

private:
    QueueHandle_t commandQueue; ///< FreeRTOS queue handle for storing commands.
};

#endif // COMMAND_SYSTEM_H
