/*
 * ----------------------------------------------------------------------------
 * ArduFlite - Advanced Flight Controller Framework
 * ----------------------------------------------------------------------------
 *
 * Project: ArduFlite
 * Description: 
 *   ArduFlite is a highly modular and real-time flight control framework for 
 *   unmanned aerial vehicles (UAVs) and gliders. Built on the ESP32 and FreeRTOS,
 *   it integrates IMU sensor fusion, cascade PID control (attitude and rate loops),
 *   servo management for multiple wing designs, and dynamic telemetry (via MQTT
 *   and Serial) along with a flexible command-line interface (CLI) for live diagnostics 
 *   and configuration.
 *
 * Author: Alexander Wasserman
 * Version: 1.0
 * Date: 08 April 2025
 *
 * License: MIT License
 * ----------------------------------------------------------------------------
 */
#include "include/ArduFlite.h"

void setup() 
{
    arduflite_init();
}

void loop() 
{
    arduflite_loop();
}