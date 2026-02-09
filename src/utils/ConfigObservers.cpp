/**
 * ConfigObservers.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 07 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Implementation of config observers that push commands when values change.
 */
#include "src/utils/ConfigObservers.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/ControlMixer.h"
#include "src/utils/Logging.h"
#include "include/ConfigKeys.h"

namespace ConfigObservers {

void registerAll()
{
    auto& reg = ConfigRegistry::instance();

    // ═══════════════════════════════════════════════════════════════════════════
    // Rate Controller PID (Inner Loop) - Hot Path @ 500Hz
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Roll rate PID - any parameter change triggers full PID rebuild
    reg.subscribe(CONFIG_KEY_RATE_ROLL_ALL, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_RATE_ROLL_PID;
        CommandSystem::instance().pushCommand(cmd);
    });

    // Pitch rate PID
    reg.subscribe(CONFIG_KEY_RATE_PITCH_ALL, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_RATE_PITCH_PID;
        CommandSystem::instance().pushCommand(cmd);
    });

    // Yaw rate PID
    reg.subscribe(CONFIG_KEY_RATE_YAW_ALL, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_RATE_YAW_PID;
        CommandSystem::instance().pushCommand(cmd);
    });

    // Rate output low-pass filter
    reg.subscribe(CONFIG_KEY_RATE_OUT_LP_ALPHA, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_RATE_OUT_ALPHA;
        CommandSystem::instance().pushCommand(cmd);
    });

    // ═══════════════════════════════════════════════════════════════════════════
    // Attitude Controller PID (Outer Loop) - Hot Path @ 100Hz
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Roll attitude PID
    reg.subscribe(CONFIG_KEY_ATT_ROLL_ALL, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_ATT_ROLL_PID;
        CommandSystem::instance().pushCommand(cmd);
    });

    // Pitch attitude PID
    reg.subscribe(CONFIG_KEY_ATT_PITCH_ALL, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_ATT_PITCH_PID;
        CommandSystem::instance().pushCommand(cmd);
    });

    // Yaw attitude PID
    reg.subscribe(CONFIG_KEY_ATT_YAW_ALL, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_ATT_YAW_PID;
        CommandSystem::instance().pushCommand(cmd);
    });

    // Attitude deadband
    reg.subscribe(CONFIG_KEY_ATT_DEADBAND, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_ATT_DEADBAND;
        CommandSystem::instance().pushCommand(cmd);
    });

    // ═══════════════════════════════════════════════════════════════════════════
    // Mixer Configuration - Medium Path (called on each RC update)
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Any mixer parameter change triggers full config reload via command queue
    reg.subscribe(CONFIG_KEY_MIX_ALL, [](const ConfigChange& change) {
        SystemCommand cmd;
        cmd.type = CMD_UPDATE_MIXER;
        CommandSystem::instance().pushCommand(cmd);
    });

    // ═══════════════════════════════════════════════════════════════════════════
    // Note: The following config categories do NOT have observers:
    //   - servo.*  : Servo config changes require initFromConfig() and reboot
    //   - imu.*    : IMU filter settings applied at init, require reboot
    //   - failsafe.*: Failsafe parameters read on-demand, no caching
    //   - crsf.*   : CRSF thresholds read on-demand, no caching
    // Pin configurations are compile-time constants in PinConfiguration.h.
    // ═══════════════════════════════════════════════════════════════════════════

    LOG_INF("Config observers registered for controller parameters.");
}

} // namespace ConfigObservers
