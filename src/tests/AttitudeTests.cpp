/**
 * AttitudeTests.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/tests/AttitudeTests.h"
#include "src/utils/Logging.h"

#include <Arduino.h>

void runAttitudeTest_wiggle(ArduFliteController &arduflite, float angle, float time)
{
    static unsigned long lastSetpointUpdate = millis();
    unsigned long currentTime = millis();
    
    if (currentTime - lastSetpointUpdate > time) 
    {
        static int state = 0;

        switch (state) 
        {
            case 0:
                arduflite.setDesiredEulerDegs(0.0f, 0.0f, 0.0f);
                LOG_INF("Test: Level attitude (0° roll)");
                state++;
                break;
            case 1:
                arduflite.setDesiredEulerDegs(angle, 0.0f, 0.0f);
                LOG_INF("Test: Roll +%f°", angle);
                state++;
                break;
            case 2:
                arduflite.setDesiredEulerDegs(0.0f, 0.0f, 0.0f);
                LOG_INF("Test: Level attitude (0° roll)");
                state++;
                break;
            case 3:
                arduflite.setDesiredEulerDegs(-angle, 0.0f, 0.0f);
                LOG_INF("Test: Roll -%f°", angle);
                state = 0;
                break;
            default:
                break;
        }
        lastSetpointUpdate = currentTime;
    }
}