/**
 * AttitudeTests.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/board/Board.h"
#include "src/tests/AttitudeTests.h"
#include "src/utils/Logging.h"

#include <Arduino.h>

void runAttitudeTest_wiggle(ArduFliteController &arduflite, float angle, float time)
{
    const auto nowMs = []() -> unsigned long {
        return static_cast<unsigned long>(
            arduflite::board::Board::instance().clock().now()
                .time_since_epoch().count() / 1000);
    };
    static unsigned long    lastSetpointUpdate  = nowMs();
    unsigned long           currentTime         = nowMs();
    unsigned long           intervalMs          = (unsigned long)(time * 1000.0f);
    AttitudeDeg             setpoint            {0.0f};

    if (currentTime - lastSetpointUpdate >= intervalMs)
    {
        static int state = 0;

        switch (state)
        {
            case 0:
                arduflite.setAttitudeSetpoint(setpoint);
                LOG_INF("Test: Level attitude (0° roll)");
                state++;
                break;
            case 1:
                setpoint.roll = angle;
                arduflite.setAttitudeSetpoint(setpoint);
                LOG_INF("Test: Roll +%f°", angle);
                state++;
                break;
            case 2:
                arduflite.setAttitudeSetpoint(setpoint);
                LOG_INF("Test: Level attitude (0° roll)");
                state++;
                break;
            case 3:
                setpoint.roll = -angle;
                arduflite.setAttitudeSetpoint(setpoint);
                LOG_INF("Test: Roll -%f°", angle);
                state = 0;
                break;
            default:
                break;
        }
        lastSetpointUpdate = currentTime;
    }
}
