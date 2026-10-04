/**
 * ArduFliteCLI.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDU_FLITE_CLI_H
#define ARDU_FLITE_CLI_H

#include "src/hal/platform/Scheduler.h"
#include <Arduino.h>
#include "src/controller/ArduFliteController.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"

class ArduFliteCLI {
public:
    /**
     * @brief Constructs the CLI.
     * @param controller A pointer to the controller, whose statistics and state we want to query.
    */
    ArduFliteCLI(ArduFliteController* controller, arduflite::estimation::InertialSubsystem* imu, ArduFliteFlashTelemetry* flashTelemetry);

    /// Inject the scheduler. Deferred like the controller's setPlatform(),
    /// because this is a global and the Board needs FreeRTOS running.
    void setScheduler(arduflite::hal::Scheduler& scheduler) { _scheduler = &scheduler; }

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
    arduflite::hal::Scheduler* _scheduler = nullptr;

    /// Sized for the deepest command handler, not the average.
    static constexpr std::uint32_t kStackBytes = 4096;

    ArduFliteController* controller;
    arduflite::estimation::InertialSubsystem* imu;
    ArduFliteFlashTelemetry* flashTelemetry;
};

#endif // ARDU_FLITE_CLI_H
