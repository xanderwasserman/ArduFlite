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

//==============================================================
// Board type selection
//==============================================================
#define BOARD_TYPE_FIREBEETLE   0
#define BOARD_TYPE_WEMOS        1

#ifndef BOARD_TYPE
#define BOARD_TYPE BOARD_TYPE_WEMOS
#endif

namespace I2CConfig 
{
    /* I2C pins for IMU */ 
#if BOARD_TYPE == BOARD_TYPE_FIREBEETLE
    constexpr int I2C_SDA_PIN           = 21;
    constexpr int I2C_SCL_PIN           = 22;
#elif BOARD_TYPE == BOARD_TYPE_WEMOS
    constexpr int I2C_SDA_PIN           = 3;
    constexpr int I2C_SCL_PIN           = 5;
#endif
    constexpr int I2C_CLOCK_SPEED       = 400000; // Set I2C clock to 400 kHz.

} // namespace I2CConfig

namespace PwmInputConfig 
{
    // PWM input pins from receiver
#if BOARD_TYPE == BOARD_TYPE_FIREBEETLE
    constexpr int ROLL_INPUT_PIN        = 34;
    constexpr int PITCH_INPUT_PIN       = 35;
    constexpr int YAW_INPUT_PIN         = 36;
    constexpr int THROTTLE_INPUT_PIN    = 39;
#elif BOARD_TYPE == BOARD_TYPE_WEMOS
    constexpr int ROLL_INPUT_PIN        = 6;
    constexpr int PITCH_INPUT_PIN       = 32;//TODO: should be 7, but pin 7 is used for the RGB LED
    constexpr int YAW_INPUT_PIN         = 8;
    constexpr int THROTTLE_INPUT_PIN    = 10;
#endif
    

} // namespace PwmInputConfig

namespace PwmOutputConfig 
{
    // Servo output pins to control surfaces
#if BOARD_TYPE == BOARD_TYPE_FIREBEETLE
    constexpr int LEFT_AIL_PIN          = 17;
    constexpr int RIGHT_AIL_PIN         = 16;
    constexpr int PITCH_PIN             = 4;
    constexpr int YAW_PIN               = 12;
#elif BOARD_TYPE == BOARD_TYPE_WEMOS
    constexpr int LEFT_AIL_PIN          = 2;
    constexpr int RIGHT_AIL_PIN         = 1;
    constexpr int PITCH_PIN             = 0;
    constexpr int YAW_PIN               = 4;
#endif

} // namespace PwmOutputConfig

namespace ButtonInputConfig 
{
    /* User Button definition */ 
#if BOARD_TYPE == BOARD_TYPE_FIREBEETLE
    constexpr int USER_BUTTON_PIN       = 27;
#elif BOARD_TYPE == BOARD_TYPE_WEMOS
    constexpr int USER_BUTTON_PIN       = 9;
#endif
} // namespace PwmOutputConfig

#endif // PIN_CONFIGURATION_H