/**
* ArduFliteIMU.h
*
* ArduFlite - Advanced Flight Controller Framework
* Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
*
* Licensed under the MIT License. See LICENSE file for details.
*/
#ifndef ARDU_FLITE_IMU_H
#define ARDU_FLITE_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>
#include <EEPROM.h>
#include <Adafruit_AHRS.h>
#include <atomic>
#include "src/orientation/FliteQuaternion.h"
#include "include/PinConfiguration.h"
#include "include/IMUConfiguration.h"

//==============================================================
// Barometer type selection
//==============================================================
#define BARO_TYPE_NONE      0
#define BARO_TYPE_BMP280    1

//==============================================================
// IMU type selection
//==============================================================
#define IMU_TYPE_MPU6500 0
#define IMU_TYPE_MPU9250 1

#if BARO_TYPE == BARO_TYPE_BMP280
#include <Adafruit_BMP280.h> // Include barometer library when using MPU9250
#endif

/**
* @defgroup OrientationModes Orientation Modes
* @{
*/
#define ORIENTATION_NORMAL                  0   ///< No axis inversion.
#define ORIENTATION_SENSOR_FLIPPED_YZ       1   ///< Invert Y and Z axes.
/** @} */
 
/// Set the current orientation mode.
#define IMU_ORIENTATION                     ORIENTATION_SENSOR_FLIPPED_YZ

/**
* @defgroup FilterTypes Filter Types
* @{
*/
#define FILTER_TYPE_MADGWICK                0   ///< Use the Madgwick filter.
#define FILTER_TYPE_KALMAN                  1   ///< Use a Kalman filter.
/** @} */

/// Set the filter update rate in Hz.
#define FILTER_UPDATE_RATE_HZ               500

/// Choose the filter type.
#define FILTER_TYPE                         FILTER_TYPE_MADGWICK

/// Minimum time step (in seconds) to avoid division by zero.
#define MIN_DT                              0.000001f

/// Maximum time step (in seconds) to prevent large integration steps.
#define MAX_DT                              0.05f

/// I2C address for the MPU-6500 (the MPU-9250 typically shares the same I2C address).
#define IMU_ADDRESS                         0x68

#define IMU_UPDATE_INTERVAL_MS              5
 
//==============================================================
// Data structures and enums
//==============================================================

/**
* @brief Enumeration for various flight states.
*/
enum FlightState {
    UNKNOWN_STATE   = 0,
    PREFLIGHT       = 1,    // The aircraft is on the ground before launch.
    INFLIGHT        = 2,    // The aircraft is in motion (launched).
    LANDED          = 3     // The aircraft has come to rest after flight.
};
 
/**
* @brief A simple 3D vector structure.
*/
struct Vector3 {
    float x;
    float y;
    float z;
};
 
/**
* @brief Structure for representing Euler angles.
*/
struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

/**
* @brief Lock-free snapshot of all IMU sensor data.
* 
* Used for double-buffering to eliminate mutex contention between
* the IMU task (writer) and control loops (readers).
*/
struct ImuSnapshot {
    Vector3         accel;          ///< Filtered accelerometer data (g)
    Vector3         gyro;           ///< Filtered gyroscope data (deg/s)
    Vector3         mag;            ///< Magnetometer data
    FliteQuaternion quat;           ///< Orientation quaternion
    EulerAngles     orientation;    ///< Euler angles (degrees)
    float           altitude;       ///< Barometric altitude (m)
    float           climbRate;      ///< Vertical speed (m/s)
    FlightState     flightState;    ///< Current flight state
    uint32_t        timestampUs;    ///< Timestamp when snapshot was captured
};

/**
* @brief Structure to hold calibration offsets for the IMU.
*/
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
 
/**
* @brief Structure for storing calibration data in EEPROM.
*/
struct StoredCalibData {
    uint32_t magic;                 //< Magic number used for data validation.
    ArduFliteIMUOffsets offsets;    //< Calibration offsets.
};

/// Magic number to validate calibration data.
static const uint32_t CALIB_MAGIC = 0xDEADBEEF;
/// EEPROM size in bytes.
static const int EEPROM_SIZE = 512;
/// EEPROM address for calibration data.
static const int CALIB_DATA_ADDR = 0;
 
//==============================================================
// ArduFliteIMU Class Declaration
//==============================================================
 
/**
* @brief ArduFliteIMU class.
*
* Provides an interface to the IMU (either MPU-6500 or MPU-9250). This class handles
* initialization, calibration, sensor updates, filtering, and orientation estimation.
* It also supports reading magnetometer data (if available) and, for the MPU-9250,
* reading barometer data from an onboard BMP280.
*/
class ArduFliteIMU {
public:
    /**
    * @brief Constructor.
    *
    * Initializes calibration offsets to zero and creates a mutex to protect access
    * to sensor data and the filter state.
    */
    ArduFliteIMU();

    /**
    * @brief Initializes the IMU hardware and sensor fusion filter.
    *
    * Configures I2C and EEPROM, initializes the IMU with calibration data,
    * sets sensor ranges, loads calibration offsets from EEPROM (or calibrates if not found),
    * and warms up the orientation filter.
    *
    * @return true if initialization is successful, false otherwise.
    */
    bool begin();

    /**
    * @brief Starts a dedicated FreeRTOS task to periodically update the IMU.
    */
    void startTask();

    /**
    * @brief Performs a calibration routine.
    *
    * This method gathers raw sensor data over a fixed period, computes the average
    * offsets, and saves these offsets to EEPROM.
    *
    * @return true if calibration is successful, false otherwise.
    */
    bool calibrate();

    /**
    * @brief Performs calibration of the barometer, with the current pressure as the 
    * ground-level reference.
    *
    * Collects raw sensor data over a fixed calibration period, computes average pressure,
    * and then saves this. This method assumes the IMU remains stationary during calibration.
    *
    * @return true if calibration is successful, false otherwise.
    */
    bool baroCalibrate();
 
    /**
    * @brief Performs self-calibration of the IMU.
    *
    * Similar to calibrate(), this method gathers data for approximately 10 seconds
    * and computes the calibration offsets.
    *
    * @return true if self-calibration succeeds, false otherwise.
    */
    bool selfCalibrate();

    /**
    * @brief Sets the calibration offsets.
    *
    * Updates the internal calibration offsets.
    *
    * @param ofs The calibration offsets to apply.
    */
    void setOffsets(const ArduFliteIMUOffsets &ofs);

    /**
    * @brief Retrieves the current calibration offsets.
    *
    * @param ofs Reference to an ArduFliteIMUOffsets structure where offsets will be stored.
    */
    void getOffsets(ArduFliteIMUOffsets &ofs) const;

    /**
    * @brief Updates the IMU sensor data and orientation filter.
    *
    * Reads raw accelerometer, gyroscope, magnetometer and barometer data, subtracts calibration offsets,
    * applies orientation transformations and low-pass filtering, updates the orientation
    * filter, and retrieves the latest quaternion and Euler angles.
    *
    * @param dt Time step in seconds.
    */
    void update(float dt);

    // Grouped getters (lock-free, read from double-buffer):
    Vector3 getAcceleration() const;
    Vector3 getGyro() const;
    Vector3 getMag() const;
    FliteQuaternion getQuaternion() const;
    EulerAngles getOrientation() const;

    /**
    * @brief Gets a complete lock-free snapshot of all IMU data.
    * 
    * This is the preferred method for control loops as it provides
    * consistent data without mutex contention. The snapshot may be
    * up to one IMU cycle (5ms) old, but timing is deterministic.
    *
    * @return ImuSnapshot containing all sensor data
    */
    ImuSnapshot getSnapshot() const;
 
    /**
    * @brief Retrieves the current flight state.
    * @return FlightState (PREFLIGHT, INFLIGHT, or LANDED).
    */
    FlightState getFlightState() const;

    /**
    * @brief Retrieves the current estimated altitude in meters.
    * @return Altitude in meters.
    */
    float getAltitude() const;

    /**
     * @brief Returns the last-computed vertical speed (m/s).
     *
     * @return Climb rate in meters per second.
     *        Positive = climbing, negative = descending.
     */
    float getClimbRate() const;

    /**
     * @brief Checks if the IMU is currently providing valid data.
     * 
     * Returns false if any of the following conditions are met:
     * - Sensor readings contain NaN or infinity values
     * - Accelerometer or gyroscope readings exceed physical limits
     * - Multiple consecutive invalid readings have occurred
     *
     * @return true if IMU data is valid and trustworthy, false otherwise.
     */
    bool isHealthy() const;

    /**
    * @brief Suspends the IMU update task.
    */
    void pauseTask();

    /**
    * @brief Resumes the IMU update task.
    */
    void resumeTask();
 
private:
    TaskHandle_t imuTaskHandle; // Handle for the IMU task

#if IMU_TYPE == IMU_TYPE_MPU9250
    MPU9250 IMU;
#else
    MPU6500 IMU;
#endif
 
#if FILTER_TYPE == FILTER_TYPE_MADGWICK
    Adafruit_Madgwick   filter;
#elif FILTER_TYPE == FILTER_TYPE_KALMAN
    Adafruit_NXPSensorFusion filter;
#endif

    // ─────────────────────────────────────────────────────────────
    // Double-buffer for lock-free reads
    // ─────────────────────────────────────────────────────────────
    // The IMU task writes to buffers[writeIndex], then swaps the index.
    // Control loops read from buffers[readIndex.load()] without any lock.
    ImuSnapshot             snapshotBuffers[2];
    std::atomic<int>        snapshotReadIndex{0};

    /**
    * @brief Publishes current sensor data to the double-buffer.
    * 
    * Called at the end of update() to make new data available to readers.
    * Uses atomic index swap for lock-free synchronization.
    */
    void publishSnapshot();
    // ─────────────────────────────────────────────────────────────
 
    ArduFliteIMUOffsets offsets;
    uint32_t            timestamp;

    // Calibrations
    float referencePressure = 1013.25f; // reference sea-level pressure as a backup
    calData calib           = {0};

    // Raw sensor readings
    float accelX            = 0.0f;
    float accelY            = 0.0f;
    float accelZ            = 0.0f;
    float gyroX             = 0.0f;
    float gyroY             = 0.0f;
    float gyroZ             = 0.0f;
    float magX              = 0.0f;
    float magY              = 0.0f;
    float magZ              = 0.0f;
    float qw                = 0.0f;
    float qx                = 0.0f;
    float qy                = 0.0f;
    float qz                = 0.0f;
    float altitude          = 0.0f;

    float accelAlpha        = IMUConfig::ACCEL_ALPHA;
    float gyroAlpha         = IMUConfig::GYRO_ALPHA;
    float magAlpha          = IMUConfig::MAG_ALPHA;
    float altiAlpha         = IMUConfig::ALTI_ALPHA;
    bool lpInitialized      = false;

    float filteredAccelX, filteredAccelY, filteredAccelZ    = 0.0f;
    float filteredGyroX, filteredGyroY, filteredGyroZ       = 0.0f;
    float filteredMagX, filteredMagY, filteredMagZ          = 0.0f;
    float filteredAltitude                                  = 0.0f;
    float lastFilteredAltitude                              = 0.0f;  ///< For vario calculation
    float climbRate                                         = 0.0f;  ///< Last computed vertical speed (m/s)
    float pitch, roll, yaw                                  = 0.0f;

    // Data structures to hold raw sensor readings.
    AccelData accelData;    //< Structure to store accelerometer data.
    GyroData  gyroData;     //< Structure to store gyroscope data.
    MagData   magData;      //< Structure to store magnetometer data.

    SemaphoreHandle_t imuMutex;

    // Flight state
    FlightState flightState             = PREFLIGHT;
    unsigned long flightStableStartTime = 0;
    unsigned long motionStartTime       = 0;

    // ─────────────────────────────────────────────────────────────────
    // Health monitoring state
    // ─────────────────────────────────────────────────────────────────
    bool    imuHealthy                  = true;  ///< Current health status
    uint8_t consecutiveFailures         = 0;     ///< Count of consecutive invalid readings

    /**
     * @brief Validates sensor readings for NaN, infinity, and range violations.
     * 
     * Updates consecutiveFailures counter and imuHealthy flag based on validation results.
     * 
     * @return true if current readings are valid, false otherwise.
     */
    bool validateSensorData();
    // ─────────────────────────────────────────────────────────────────

#if BARO_TYPE == BARO_TYPE_BMP280
    Adafruit_BMP280 bmp280;
#endif

    /**
    * @brief Static FreeRTOS task function for IMU updates.
    *
    * @param parameters A pointer to the current ArduFliteIMU instance.
    */
    static void imuTask(void* parameters);

    /**
    * @brief Loads calibration offsets from EEPROM.
    *
    * Retrieves stored calibration data and verifies its validity using a magic number.
    *
    * @param dest Reference to an ArduFliteIMUOffsets structure where the offsets will be stored.
    * @return true if valid calibration data is loaded, false otherwise.
    */
    bool loadOffsetsFromEEPROM(ArduFliteIMUOffsets &dest);

    /**
    * @brief Saves calibration offsets to EEPROM.
    *
    * Writes the provided calibration offsets to EEPROM and commits the changes.
    *
    * @param ofs The calibration offsets to save.
    */
    void saveOffsetsToEEPROM(const ArduFliteIMUOffsets &ofs);

    /**
    * @brief Applies sensor orientation transformations.
    *
    * Adjusts raw sensor data according to the defined IMU orientation.
    */
    void applyOrientation();

    /**
    * @brief Applies calibration offsets from EEPROM.
    *
    * Loads calibration offsets from EEPROM and applies them. If no valid data is found,
    * self-calibration may be initiated.
    *
    * @return true if calibration offsets are successfully applied, false otherwise.
    */
    bool applyCalibrations();

    /**
    * @brief Initializes the orientation filter.
    *
    * Warms up the filter by running several update iterations so that stable orientation
    * estimates are produced.
    */
    void initFilter();

    /**
    * @brief Applies low-pass filters to the raw sensor data.
    *
    * If the filters are not yet initialized, raw sensor data is assigned directly to the
    * filtered variables. Otherwise, an exponential moving average is applied.
    */
    void applyLowPassFilters();

    /**
    * @brief Updates the flight state based on filtered sensor data.
    */
    void updateFlightState();
};

#endif // ARDU_FLITE_IMU_H
 