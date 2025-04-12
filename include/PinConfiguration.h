/**
* PinConfiguration.h
*
* ArduFlite - Advanced Flight Controller Framework
* Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
*
* Licensed under the MIT License. See LICENSE file for details.
*/
#ifndef PIN_CONFIGURATION_H
#define PIN_CONFIGURATION_H

namespace I2CConfig {

    // I2C pins for IMU
    constexpr int I2C_SDA_PIN           = 21;
    constexpr int I2C_SCL_PIN           = 22;
    constexpr int I2C_CLOCK_SPEED       = 400000; // Set I2C clock to 400 kHz.

} // namespace I2CConfig

namespace PwmInputConfig {

    // PWM input pins from receiver
    constexpr int ROLL_INPUT_PIN        = 34;
    constexpr int PITCH_INPUT_PIN       = 35;
    constexpr int YAW_INPUT_PIN         = 36;
    constexpr int THROTTLE_INPUT_PIN    = 39;

} // namespace PwmInputConfig

namespace PwmOutputConfig {

    // Servo output pins to control surfaces
    constexpr int LEFT_AIL_PIN          = 17;
    constexpr int RIGHT_AIL_PIN         = 16;
    constexpr int PITCH_PIN             = 4;
    constexpr int YAW_PIN               = 12;

} // namespace PwmOutputConfig

namespace ButtonInputConfig {

    // Calibration Button definition
    constexpr int USER_BUTTON_PIN       = 27;

} // namespace PwmOutputConfig

#endif // PIN_CONFIGURATION_H