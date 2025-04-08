#ifndef ARDU_FLITE_CLI_H
#define ARDU_FLITE_CLI_H

#include <Arduino.h>
#include "ArduFliteController.h"

class ArduFliteCLI {
public:
    /**
     * @brief Constructs the CLI.
     * @param controller A pointer to the controller, whose statistics and state we want to query.
     */
    ArduFliteCLI(ArduFliteController* controller);

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
};

#endif // ARDU_FLITE_CLI_H
