/**
 * AttitudeTests.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/tests/AttitudeTests.h"
#include <Arduino.h>

#define WIGGLE_ANGLE    30
#define WIGGLE_TIME     3000 //3s

void runAttitudeTest_wiggle(ArduFliteController &arduflite)
{
    static unsigned long lastSetpointUpdate = millis();
    unsigned long currentTime = millis();
    
    if (currentTime - lastSetpointUpdate > WIGGLE_TIME) 
    {
        static int state = 0;
        int angle = WIGGLE_ANGLE;

        switch (state) 
        {
            case 0:
                arduflite.setDesiredEulerDegs(-2.0f, 0.0f, 0.0f);
                Serial.println("Test: Level attitude (0° roll)");
                state++;
                break;
            case 1:
                arduflite.setDesiredEulerDegs(-2.0f, angle, 0.0f);
                Serial.printf("Test: Roll +%d°\n", angle);
                state++;
                break;
            case 2:
                arduflite.setDesiredEulerDegs(-2.0f, 0.0f, 0.0f);
                Serial.println("Test: Level attitude (0° roll)");
                state++;
                break;
            case 3:
                arduflite.setDesiredEulerDegs(-2.0f, -angle, 0.0f);
                Serial.printf("Test: Roll -%d°\n", angle);
                state = 0;
                break;
            default:
                break;
        }
        lastSetpointUpdate = currentTime;
    }
}