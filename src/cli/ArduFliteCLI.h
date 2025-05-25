/**
 * ArduFliteCLI.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDU_FLITE_CLI_H
#define ARDU_FLITE_CLI_H

#include <Arduino.h>
#include "src/controller/ArduFliteController.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"

class ArduFliteCLI {
public:
    /**
     * @brief Constructs the CLI.
     * @param controller A pointer to the controller, whose statistics and state we want to query.
     */
    ArduFliteCLI(ArduFliteController* controller, ArduFliteIMU* imu, ArduFliteFlashTelemetry* flashTelemetry);

    /**
     * @brief Starts the CLI task.
     */
    void startTask();

    /**
     * @brief The CLI task function.
     * This function reads commands from Serial and prints corresponding output.
     */
    static void cliTask(void* parameters);

private:
    ArduFliteController* controller;
    ArduFliteIMU* imu;
    ArduFliteFlashTelemetry* flashTelemetry;
};

#endif // ARDU_FLITE_CLI_H
