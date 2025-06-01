/**
 * ConfigData.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 30 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONFIG_DATA_H
#define CONFIG_DATA_H

#include "src/controller/ArduFliteController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/controller/ArduFliteAttitudeController.h"

struct FilterGains 
{
    float alpha;
};

struct ConfigData 
{
    PIDConfig   attitude_pidRoll;
    PIDConfig   attitude_pidPitch;
    PIDConfig   attitude_pidYaw;
    PIDConfig   rate_pidRoll;
    PIDConfig   rate_pidPitch;
    PIDConfig   rate_pidYaw;
    float       filter_alpha;

    // Pull fresh values from Controller
    void update(const ArduFliteController &controller)  
    { 
        //TODO: actually implement this. Need the necessary methos in the controllers
    }
};

#endif // CONFIG_DATA_H
