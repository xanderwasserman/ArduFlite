/**
 * ConfigObservers.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 07 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Sets up config observers that push commands when values change.
 *        This bridges the config system with the command system.
 */
#ifndef CONFIG_OBSERVERS_H
#define CONFIG_OBSERVERS_H

namespace ConfigObservers {

/**
 * @brief Register all config observers.
 * 
 * Subscribes to config key patterns and pushes appropriate commands
 * when values change. Safe for hot-path parameters since commands
 * are processed in the main loop via CommandSystem.
 * 
 * Call once after ConfigRegistry::init() and before controllers start.
 */
void registerAll();

} // namespace ConfigObservers

#endif // CONFIG_OBSERVERS_H
