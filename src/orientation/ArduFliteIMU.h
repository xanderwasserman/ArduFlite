#ifndef ARDU_FLITE_IMU_H
#define ARDU_FLITE_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>
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

#define IMU_ORIENTATION ORIENTATION_SENSOR_FLIPPED_YZ

#define FILTER_TYPE_MADGWICK 0
#define FILTER_TYPE_KALMAN 1

#define FILTER_UPDATE_RATE_HZ 500
#define FILTER_TYPE FILTER_TYPE_KALMAN

#define MIN_DT 0.000001f  // Minimum dt to avoid division by zero issues.
#define MAX_DT 0.05f      // Maximum dt to prevent large integration steps.

#define IMU_ADDRESS 0x68  // MPU-6500 address

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

    float getAccelX() const { return filteredAccelX; }
    float getAccelY() const { return filteredAccelY; }
    float getAccelZ() const { return filteredAccelZ; }
    float getGyroX()  const { return filteredGyroX; }
    float getGyroY()  const { return filteredGyroY; }
    float getGyroZ()  const { return filteredGyroZ; }
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

#if FILTER_TYPE == FILTER_TYPE_MADGWICK
    Adafruit_Madgwick filter;
#elif FILTER_TYPE == FILTER_TYPE_KALMAN
    Adafruit_NXPSensorFusion filter;
#endif
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

    // Smoothing factors (0 < alpha <= 1). Lower values are smoother but slower.
    float accelAlpha = 0.05f;  
    float gyroAlpha  = 0.8f;

    // Filtered (smoothed) sensor values
    float filteredAccelX, filteredAccelY, filteredAccelZ = 0.0f;
    float filteredGyroX,  filteredGyroY,  filteredGyroZ = 0.0f;
    float filteredMagX,  filteredMagY,  filteredMagZ = 0.0f;

    // The final Euler angles (in degrees)
    float pitch = 0.0f;
    float roll  = 0.0f;
    float yaw   = 0.0f;

    AccelData accelData;
    GyroData  gyroData;
    MagData   magData;

    void applyOrientation();
    bool applyCalibrations();
    void initFilter();
    void applyLowPassFilters();
};

#endif // ARDU_FLITE_IMU_H
