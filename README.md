# ArduFlite
A simple flight stabilisation controller for a glider using a FireBeetle 2 ESP32-E, a 6DOF IMU (MPU-6500), and a couple 9g micro servos.

The software supports the correction of all three axis; roll, pitch, and yaw. However, yaw might not be perfect, as there is no magnetometer in the IMU. This could be a future improvement.

# References
- [FireBeetle 2 ESP32-E](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654#target_3)
- [Adafruit_AHRS](https://github.com/adafruit/Adafruit_AHRS/tree/master)
- [FastIMU](https://github.com/LiquidCGS/FastIMU/tree/main)
