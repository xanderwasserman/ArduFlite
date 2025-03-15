#ifndef ARDU_FLITE_IMU_H
#define ARDU_FLITE_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "FastIMU.h"
#include <EEPROM.h>
#include <Adafruit_AHRS.h>

//////////////////////////////////////////////////////////////
// ORIENTATION MODES
//////////////////////////////////////////////////////////////
// 0) NORMAL_ORIENTATION          => No axis inversion
// 1) SENSOR_FLIPPED_YZ           => Invert Y, Z
//////////////////////////////////////////////////////////////

#define ORIENTATION_NORMAL                0
#define ORIENTATION_SENSOR_FLIPPED_YZ     1

// SELECT YOUR MODE HERE:
#define IMU_ORIENTATION ORIENTATION_SENSOR_FLIPPED_YZ
// ^^^ Change this to 1 as needed ^^^

#define IMU_ADDRESS 0x68  // MPU-6500 address

#define FILTER_UPDATE_RATE_HZ 100

struct ArduFliteIMUOffsets {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float magX;
    float magY;
    float magZ;
};

struct StoredCalibData {
    uint32_t magic;
    ArduFliteIMUOffsets offsets;
};

static const uint32_t CALIB_MAGIC = 0xDEADBEEF;
static const int EEPROM_SIZE = 512;
static const int CALIB_DATA_ADDR = 0;

class ArduFliteIMU {
public:
    ArduFliteIMU();

    bool begin();
    bool calibrate();
    bool selfCalibrate();
    void setOffsets(const ArduFliteIMUOffsets &ofs);
    void getOffsets(ArduFliteIMUOffsets &ofs) const;
    void update(float dt);

    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }
    float getGyroX()  const { return gyroX; }
    float getGyroY()  const { return gyroY; }
    float getGyroZ()  const { return gyroZ; }
    float getMagX()  const { return magX; }
    float getMagY()  const { return magY; }
    float getMagZ()  const { return magZ; }
    float getQw()  const { return qw; }
    float getQx()  const { return qx; }
    float getQy()  const { return qy; }
    float getQz()  const { return qz; }
    float getPitch()  const { return pitch; }
    float getRoll()   const { return roll; }
    float getYaw()    const { return yaw; }

private:
    MPU6500 IMU;
    Adafruit_Madgwick filter;
    ArduFliteIMUOffsets offsets;

    bool loadOffsetsFromEEPROM(ArduFliteIMUOffsets &dest);
    void saveOffsetsToEEPROM(const ArduFliteIMUOffsets &ofs);

    uint32_t timestamp;

    calData calib = {0};

    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    float magX = 0.0f;
    float magY = 0.0f;
    float magZ = 0.0f;
    float qw = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;

    // The final Euler angles (in degrees)
    float pitch = 0.0f;
    float roll  = 0.0f;
    float yaw   = 0.0f;

    float alpha = 0.98f; // (Unused now with Madgwick, but kept for legacy)

    AccelData accelData;
    GyroData  gyroData;
    MagData   magData;

    void applyOrientation();
    bool applyCalibrations();
    void initMadgwickFilter();
};

#endif // ARDU_FLITE_IMU_H
