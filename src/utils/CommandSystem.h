/**
 * CommandSystem.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 16 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef COMMAND_SYSTEM_H
#define COMMAND_SYSTEM_H

#include "src/controller/ArduFliteController.h"
#include "src/core/FlightTypes.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/state/StateManagement.h"
#include "src/hal/device/RcLink.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/**
 * @brief Command types for system commands.
 */
enum SystemCommandType 
{
    CMD_NONE = 0,
    CMD_RESET,
    CMD_CALIBRATE,
    CMD_SET_ARM,
    CMD_SET_THROTTLE_CUT,
    CMD_SET_MODE,
    CMD_SET_MISSION,
    CMD_SET_SETPOINT,               // Set full roll/pitch/yaw setpoint (from mixer)
    CMD_SET_SETPOINT_ROLL,          // Set roll setpoint (from receiver)
    CMD_SET_SETPOINT_PITCH,         // Set pitch setpoint (from receiver)
    CMD_SET_SETPOINT_YAW,           // Set yaw setpoint (from receiver)
    CMD_SET_SETPOINT_THROTTLE,      // Set throttle setpoint (from receiver)
    
    // Config update commands (pushed by observers, processed in main loop)
    CMD_UPDATE_RATE_ROLL_PID,
    CMD_UPDATE_RATE_PITCH_PID,
    CMD_UPDATE_RATE_YAW_PID,
    CMD_UPDATE_RATE_OUT_ALPHA,
    CMD_UPDATE_ATT_ROLL_PID,
    CMD_UPDATE_ATT_PITCH_PID,
    CMD_UPDATE_ATT_YAW_PID,
    CMD_UPDATE_ATT_DEADBAND,
    CMD_UPDATE_MIXER,
};

/**
 * @brief Structure representing a system command.
 */
struct SystemCommand 
{
    SystemCommandType               type;

    ArduFliteMode                   mode;

    // for CMD_SET_SETPOINT* commands
    /**
     * @brief What the mixer produced, and WHICH quantity it is.
     *
     * The kind travels with the value on purpose. If the mixer picked a
     * scaling from the flight mode and CommandSystem re-read the mode to pick a
     * setter, that would be two reads of something that can change in between —
     * and a rate-scaled setpoint could reach the attitude setter. The value
     * says what it is, and the mode is consulted once.
     */
    enum class SetpointKind : std::uint8_t { Attitude, Rate, Manual };

    SetpointKind                    setpointKind = SetpointKind::Attitude;
    float                           setpointRoll  = 0.0f;
    float                           setpointPitch = 0.0f;
    float                           setpointYaw   = 0.0f;

    // for any generic value
    float                           value;
    bool                            x_value;
};

/**
 * @class CommandSystem
 * @brief Thread-safe singleton queue for SystemCommand.
 *
 * This class implements a thread-safe command system using a FreeRTOS queue.
 * Any module can push a SystemCommand onto this queue.
 * The main loop (or another designated part of your code) should periodically call
 * processCommands() to handle and execute any pending commands, using the provided
 * pointers to ArduFliteController and arduflite::estimation::InertialSubsystem.
 * 
 * Use CommandSystem::instance() to access the one and only instance.
 */
class CommandSystem 
{
public:
    /**
     * @brief Get the single shared instance.
     * @return reference to the CommandSystem
     */
    static CommandSystem& instance();

    /**
     * @brief Pushes a command onto the queue (best-effort, non-blocking).
     *
     * Producers are never blocked: if the queue is full the command is dropped and a
     * rate-limited warning is logged. This applies to ALL command types, including
     * safety-critical ones (CMD_SET_ARM, CMD_SET_THROTTLE_CUT, failsafe mode/setpoint),
     * so delivery is best-effort. In practice the queue only saturates if the single
     * consumer (the main loop) stalls — and note that failsafe coincides with the RC
     * setpoint stream stopping, so the queue is draining when failsafe commands arrive.
     *
     * Task context only: uses xQueueSend(), not the FromISR variant — do not call from
     * an ISR.
     *
     * @param cmd The SystemCommand to push.
     * @return true if the command was enqueued, false if it was dropped.
     */
    bool pushCommand(const SystemCommand &cmd);

    /**
     * @brief Processes up to MAX_COMMANDS_PER_TICK (10) pending commands per call.
     *
     * Dequeues pending commands (non-blocking) and executes them, draining at most
     * MAX_COMMANDS_PER_TICK per call to bound per-tick latency; any remainder is handled
     * on subsequent calls. Accepts pointers to an ArduFliteController and arduflite::estimation::InertialSubsystem so
     * that command processing may invoke methods on these objects.
     *
     * @note Single-consumer only: must be called from exactly one task (the main loop).
     *       The command handlers mutate shared controller state assuming no concurrent
     *       processCommands() call.
     *
     * @param controller Pointer to the ArduFliteController instance.
     * @param imu Pointer to the arduflite::estimation::InertialSubsystem instance.
     * @param rcLink Pointer to the RC link, for preflight checks (may be nullptr).
     */
    void processCommands(ArduFliteController *controller, arduflite::estimation::InertialSubsystem *imu,
                         arduflite::device::RcLink *rcLink = nullptr);

private:
    QueueHandle_t commandQueue_;  ///< FreeRTOS queue handle

    /// Private ctor/dtor for singleton enforcement
    CommandSystem();
    ~CommandSystem();

    /// No copies or moves
    CommandSystem(const CommandSystem&)            = delete;
    CommandSystem& operator=(const CommandSystem&) = delete;
    CommandSystem(CommandSystem&&)                 = delete;
    CommandSystem& operator=(CommandSystem&&)      = delete;
};

#endif // COMMAND_SYSTEM_H
