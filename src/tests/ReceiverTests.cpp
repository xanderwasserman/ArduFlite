/**
 * ReceiverTests.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
 #include "src/tests/ReceiverTests.h"
 #include "src/utils/Logging.h"

#include <Arduino.h>

void runReceiverTest_print(ArduFlitePwmReceiver &pilotReceiver, size_t channels)
{
    static unsigned long lastUpdate = millis();
    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdate > 1000) 
    {
        // Read and print normalized input values for each channel.
        for (size_t i = 0; i < channels; i++) {
            float value = pilotReceiver.getNormalizedValue(i);
            LOG_INF("Channel %d: %f", i+1, value);
        }
        lastUpdate = currentTime;
    }
}