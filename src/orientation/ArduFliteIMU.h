#ifndef ARDU_FLITE_IMU_H
#define ARDU_FLITE_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>
#include <EEPROM.h>
#include <Adafruit_AHRS.h>
#include "src/orientation/FliteQuaternion.h"

/**
 * @defgroup OrientationModes Orientation Modes
 * @{
 * Defines the different orientation modes for the IMU.
 */
#define ORIENTATION_NORMAL                  0   ///< No axis inversion.
#define ORIENTATION_SENSOR_FLIPPED_YZ       1   ///< Invert Y and Z axes.
/** @} */

/// Set the current orientation mode.
#define IMU_ORIENTATION                     ORIENTATION_SENSOR_FLIPPED_YZ

/**
 * @defgroup FilterTypes Filter Types
 * @{
 * Defines the filter types available for sensor fusion.
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

/// I2C address for the MPU-6500.
#define IMU_ADDRESS                         0x68

/**
 * @brief A simple 3D vector structure.
 */
struct Vector3 {
    float x; ///< X component.
    float y; ///< Y component.
    float z; ///< Z component.
};

/**
 * @brief Structure for representing Euler angles.
 */
struct EulerAngles {
    float roll;  ///< Roll angle.
    float pitch; ///< Pitch angle.
    float yaw;   ///< Yaw angle.
};

/**
 * @brief Structure to hold calibration offsets for the IMU.
 */
struct ArduFliteIMUOffsets {
    float accelX; ///< Accelerometer X offset.
    float accelY; ///< Accelerometer Y offset.
    float accelZ; ///< Accelerometer Z offset.
    float gyroX;  ///< Gyroscope X offset.
    float gyroY;  ///< Gyroscope Y offset.
    float gyroZ;  ///< Gyroscope Z offset.
    float magX;   ///< Magnetometer X offset.
    float magY;   ///< Magnetometer Y offset.
    float magZ;   ///< Magnetometer Z offset.
};

/**
 * @brief Structure for storing calibration data in EEPROM.
 */
struct StoredCalibData {
    uint32_t magic;             ///< Magic number used for data validation.
    ArduFliteIMUOffsets offsets;///< Calibration offsets.
};

/// Magic number to validate calibration data.
static const uint32_t CALIB_MAGIC = 0xDEADBEEF;
/// EEPROM size in bytes.
static const int EEPROM_SIZE = 512;
/// EEPROM address for calibration data.
static const int CALIB_DATA_ADDR = 0;

/**
 * @brief ArduFliteIMU class.
 *
 * Provides an interface to the IMU (e.g., MPU-6500) including initialization,
 * calibration, sensor updates, low-pass filtering, and orientation estimation.
 * Grouped getters return filtered accelerometer, gyroscope, magnetometer,
 * quaternion, and Euler angle data. Access to shared sensor data and filter state
 * is protected by a FreeRTOS mutex.
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
     * @brief Performs a calibration routine.
     *
     * This method gathers raw sensor data over a fixed period, computes the average
     * offsets, and saves these offsets to EEPROM.
     *
     * @return true if calibration is successful, false otherwise.
     */
    bool calibrate();

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
     * Reads raw accelerometer and gyroscope data, subtracts calibration offsets,
     * applies orientation transformations and low-pass filtering, updates the orientation
     * filter, and retrieves the latest quaternion and Euler angles.
     *
     * @param dt Time step in seconds.
     */
    void update(float dt);

    // Grouped getters for sensor data:

    /**
     * @brief Retrieves filtered accelerometer data.
     *
     * @return Vector3 containing the filtered acceleration values.
     */
    Vector3 getAcceleration() const;

    /**
     * @brief Retrieves filtered gyroscope data.
     *
     * @return Vector3 containing the filtered gyroscope values.
     */
    Vector3 getGyro() const;

    /**
     * @brief Retrieves magnetometer data.
     *
     * @return Vector3 containing the magnetometer values.
     */
    Vector3 getMag() const;

    /**
     * @brief Retrieves the current orientation as a quaternion.
     *
     * @return FliteQuaternion representing the current orientation.
     */
    FliteQuaternion getQuaternion() const;

    /**
     * @brief Retrieves the current orientation as Euler angles.
     *
     * @return EulerAngles containing the roll, pitch, and yaw angles.
     */
    EulerAngles getOrientation() const;

private:
    /// IMU hardware instance.
    MPU6500 IMU;

#if FILTER_TYPE == FILTER_TYPE_MADGWICK
    /// Orientation filter instance using the Madgwick algorithm.
    Adafruit_Madgwick filter;
#elif FILTER_TYPE == FILTER_TYPE_KALMAN
    /// Orientation filter instance using a Kalman filter.
    Adafruit_NXPSensorFusion filter;
#endif

    /// Calibration offsets for sensor data.
    ArduFliteIMUOffsets offsets;

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

    /// Timestamp used for sensor data updates.
    uint32_t timestamp;

    /// Calibration data for the IMU.
    calData calib = {0};

    // Raw sensor values.
    float accelX = 0.0f; ///< Raw accelerometer X value.
    float accelY = 0.0f; ///< Raw accelerometer Y value.
    float accelZ = 0.0f; ///< Raw accelerometer Z value.
    float gyroX = 0.0f;  ///< Raw gyroscope X value.
    float gyroY = 0.0f;  ///< Raw gyroscope Y value.
    float gyroZ = 0.0f;  ///< Raw gyroscope Z value.
    float magX = 0.0f;   ///< Raw magnetometer X value.
    float magY = 0.0f;   ///< Raw magnetometer Y value.
    float magZ = 0.0f;   ///< Raw magnetometer Z value.
    float qw = 0.0f;     ///< Quaternion W component.
    float qx = 0.0f;     ///< Quaternion X component.
    float qy = 0.0f;     ///< Quaternion Y component.
    float qz = 0.0f;     ///< Quaternion Z component.

    // Smoothing factors for low-pass filters (0 < alpha <= 1).
    float accelAlpha = 0.05f;  ///< Accelerometer low-pass filter coefficient.
    float gyroAlpha  = 0.9f;   ///< Gyroscope low-pass filter coefficient.
    bool lpInitialized = false;///< Indicates whether the low-pass filters have been initialized.

    // Filtered (smoothed) sensor values.
    float filteredAccelX; ///< Filtered accelerometer X value.
    float filteredAccelY; ///< Filtered accelerometer Y value.
    float filteredAccelZ; ///< Filtered accelerometer Z value.
    float filteredGyroX;  ///< Filtered gyroscope X value.
    float filteredGyroY;  ///< Filtered gyroscope Y value.
    float filteredGyroZ;  ///< Filtered gyroscope Z value.
    float filteredMagX;   ///< Filtered magnetometer X value.
    float filteredMagY;   ///< Filtered magnetometer Y value.
    float filteredMagZ;   ///< Filtered magnetometer Z value.

    // Final Euler angles (in degrees) computed by the filter.
    float pitch = 0.0f; ///< Computed pitch angle.
    float roll  = 0.0f; ///< Computed roll angle.
    float yaw   = 0.0f; ///< Computed yaw angle.

    // Data structures to hold raw sensor readings.
    AccelData accelData; ///< Structure to store accelerometer data.
    GyroData  gyroData;  ///< Structure to store gyroscope data.
    MagData   magData;   ///< Structure to store magnetometer data.

    /// Mutex to protect access to sensor data and filter state.
    SemaphoreHandle_t imuMutex;

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
};

#endif // ARDU_FLITE_IMU_H
