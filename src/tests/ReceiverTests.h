/**
 * ReceiverTests.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
 #ifndef RECEIVER_TESTS_H
#define RECEIVER_TESTS_H

#include "src/receiver/pwm/ArduFlitePwmReceiver.h"

/**
* @brief Runs a test sequence for the PWM receiver
* That prints the PWM inputs.
* @param pilotReceiver Reference to the ArduFlitePwmReceiver instance.
*/
void runReceiverTest_print(ArduFlitePwmReceiver &pilotReceiver, size_t channels);

#endif //RECEIVER_TESTS_H