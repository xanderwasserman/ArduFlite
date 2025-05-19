/**
 * MissionConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 10 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef MISSION_CONFIGURATION_H
#define MISSION_CONFIGURATION_H

#include "src/mission_planner/MissionPlanner.h"

namespace MissionConfig 
{
    constexpr MissionPlanner::Step MISSION_STEPS[] = {
        {  0.0f,  5.0f,  0.0f, 2000 },  // climb 5° for 2 s
        { 20.0f,  0.0f,  0.0f, 1000 },  // roll +10° for 1 s
        {  0.0f,  0.0f,  0.0f, 1000 },  // level for 0.5 s
        {-20.0f,  0.0f,  0.0f, 1000 },  // roll –10° for 1 s
        {  0.0f,  0.0f,  0.0f, 1000 },  // level for 1 s
    };
} // namespace MissionConfig

#endif // MISSION_CONFIGURATION_H