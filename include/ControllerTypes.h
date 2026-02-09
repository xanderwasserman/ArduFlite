/**
 * ControllerTypes.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 07 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Common type definitions used across controller classes.
 */
#ifndef CONTROLLER_TYPES_H
#define CONTROLLER_TYPES_H

/**
 * @brief Identifies which control loop axis to configure.
 */
enum ControlLoopType
{
    ATTITUDE_ROLL_LOOP = 0,
    ATTITUDE_PITCH_LOOP,
    ATTITUDE_YAW_LOOP,
    RATE_ROLL_LOOP,
    RATE_PITCH_LOOP,
    RATE_YAW_LOOP
};

#endif // CONTROLLER_TYPES_H
