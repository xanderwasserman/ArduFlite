/**
 * ArduFliteIMU.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @file ArduFliteIMU.cpp
 * @brief Implementation of the ArduFliteIMU class.
 *
 * This class interfaces with the IMU sensor (e.g., MPU-6500), handling initialization,
 * calibration, sensor updates, filtering, and orientation estimation. Shared sensor data
 * is protected by a FreeRTOS mutex.
 */

#include "src/orientation/ArduFliteIMU.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

#include <esp_task_wdt.h>  // ESP32 hardware watchdog
#include <math.h>

/**
* @brief Constructor.
*
* Initializes calibration offsets to zero and creates a mutex to protect the IMU data.
*/
ArduFliteIMU::ArduFliteIMU() 
{
    // Initialize calibration offsets to zero.
    offsets = {
        0.0f,  // accelX
        0.0f,  // accelY
        0.0f,  // accelZ
        0.0f,  // gyroX
        0.0f,  // gyroY
        0.0f,  // gyroZ
        0.0f,  // magX
        0.0f,  // magY
        0.0f,  // magZ
    };

    // Create the mutex for protecting sensor data.
    imuMutex = xSemaphoreCreateMutex();
    if (imuMutex == NULL) 
    {
        LOG_ERR("Failed to create IMU mutex!");
    }
}
 
/**
* @brief Initializes the IMU.
*
* Configures the I2C interface, initializes the IMU with calibration data, sets sensor ranges,
* loads calibration offsets from EEPROM (or calibrates if none exist), and warms up the orientation filter.
*
* @return true if the IMU is successfully initialized, false otherwise.
*/
bool ArduFliteIMU::begin() 
{
    // For ESP32, initialize EEPROM.
    EEPROM.begin(EEPROM_SIZE);
    Wire.begin(I2CConfig::I2C_SDA_PIN, I2CConfig::I2C_SCL_PIN);
    Wire.setClock(I2CConfig::I2C_CLOCK_SPEED); 

    // Initialize IMU with calibration data.
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) 
    {
        LOG_ERR("FastIMU init error: %d", err);
        return false;
    }

    // Set sensor ranges.
    err = IMU.setGyroRange(300);
    err = IMU.setAccelRange(4);
    if (err == -1) 
    {
        LOG_ERR("Error setting IMU sensor ranges: %d", err);
        return false;
    }

    // Load calibration offsets from EEPROM; if unavailable, perform self-calibration.
    if (!applyCalibrations()) 
    {
        LOG_ERR("FastIMU failed to apply calibrations!");
        return false;
    }

#if BARO_TYPE == BARO_TYPE_BMP280
    // Initialize BMP280 barometer. Change the address if necessary.
    if (!bmp280.begin(0x76)) 
    {
        LOG_ERR("Failed to initialize BMP280 barometer!");
        return false;
    } 

    // Immediately set our ground‑level reference pressure:
    if (!baroCalibrate())   return false;
    LOG_INF("BMP280 barometer initialized.");
    
#endif

    // Initialize and warm up the orientation filter.
    initFilter();

#if IMU_TYPE == IMU_TYPE_MPU9250
    LOG_INF("FastIMU (MPU-9250) initialized!");
#else
    LOG_INF("FastIMU (MPU-6500) initialized!");
#endif

    // Start the dedicated IMU update task.
    startTask();

    return true;
}

/**
 * @brief Starts the dedicated IMU update task.
 *
 * This function creates a FreeRTOS task that periodically calls update() on the IMU
 * at the period defined by IMU_UPDATE_INTERVAL_MS (in milliseconds). This task runs
 * independently from the control loop tasks.
 */
void ArduFliteIMU::startTask() 
{
    // Create an IMU task that runs at the desired update frequency.
    // For example, if you want to update every 5 ms (~200Hz), use a period of 5ms.
    const TickType_t taskPeriod = pdMS_TO_TICKS(5); 
    xTaskCreate(imuTask, "IMU Task", 4096, this, 4, &imuTaskHandle);
}

/**
 * @brief Suspends the dedicated IMU update task.
 *
 * Suspends the IMU task so that sensor readings are paused during critical operations,
 * such as calibration. This ensures that no new sensor data is processed while changes
 * are being made.
 */
void ArduFliteIMU::pauseTask() 
{
    if (imuTaskHandle != NULL) 
    {
        // Unsubscribe from watchdog BEFORE suspending to prevent false triggers
        esp_task_wdt_delete(imuTaskHandle);
        vTaskSuspend(imuTaskHandle);
        LOG_INF("IMU Task paused (WDT unsubscribed).");
    }
}
 
 /**
  * @brief Resumes the dedicated IMU update task.
  *
  * Resumes the previously suspended IMU task, allowing sensor data to be updated again.
  */
void ArduFliteIMU::resumeTask() 
{
    if (imuTaskHandle != NULL) 
    {
        vTaskResume(imuTaskHandle);
        // Re-subscribe to watchdog after resuming
        esp_task_wdt_add(imuTaskHandle);
        LOG_INF("IMU Task resumed (WDT re-subscribed).");
    }
}

/**
 * @brief FreeRTOS task function for IMU updates.
 *
 * This static function is the entry point for the IMU update task. It continuously
 * calculates the time delta (dt) between iterations, calls the update() method with
 * this dt, and then delays until the next scheduled execution time.
 *
 * @param parameters Pointer to the ArduFliteIMU instance.
 */
void ArduFliteIMU::imuTask(void* parameters) 
{
    ArduFliteIMU* imuInstance = static_cast<ArduFliteIMU*>(parameters);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long lastMicros = micros();

    // Register this task with hardware watchdog (must be done from within the task)
    esp_task_wdt_add(NULL);  // NULL = current task

    // Define the task period; here, 5ms corresponds to roughly 200Hz.
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_UPDATE_INTERVAL_MS);

    while (true) 
    {
        // Reset hardware watchdog - proves this task is alive
        esp_task_wdt_reset();

        unsigned long currentMicros = micros();
        unsigned long dtMicro = currentMicros - lastMicros;
        lastMicros = currentMicros;

        float dt = dtMicro / 1000000.0f;
        if (dt < 1e-3f) dt = 1e-3f;

        imuInstance->update(dt);

        // Delay until the next iteration.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
 
/**
* @brief Warms up the orientation filter.
*
* Runs a number of update iterations to allow the orientation filter to settle, ensuring
* stable results when normal operation begins.
*/
void ArduFliteIMU::initFilter() 
{   
    // Begin the filter with an initial sample frequency.
    filter.begin(FILTER_UPDATE_RATE_HZ);

    // Warm up the filter by updating it for 2000 iterations.
    unsigned long lastMicros = micros();
    for (int i = 0; i < 2000; i++) 
    {
        // Calculate delta time in seconds.
        unsigned long currentMicros = micros();
        float dt = (currentMicros - lastMicros) / 1000000.0f;
        lastMicros = currentMicros;

        // Clamp dt within acceptable bounds.
        if (dt < MIN_DT) dt = MIN_DT;
        if (dt > MAX_DT) dt = MAX_DT;

        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        if (IMU.hasMagnetometer()) 
        {
            IMU.getMag(&magData);
        }

        // Remove calibration offsets.
        accelX = accelData.accelX - offsets.accelX;
        accelY = accelData.accelY - offsets.accelY;
        accelZ = accelData.accelZ - offsets.accelZ;

        gyroX = gyroData.gyroX - offsets.gyroX;
        gyroY = gyroData.gyroY - offsets.gyroY;
        gyroZ = gyroData.gyroZ - offsets.gyroZ; 

        // Apply any sensor orientation transformations.
        applyOrientation();

        // Update low-pass filters on the sensor data.
        applyLowPassFilters();

#if FILTER_TYPE == FILTER_TYPE_MADGWICK

    #if IMU_TYPE == IMU_TYPE_MPU9250
        // Update the orientation filter using the filtered sensor values.
        filter.update(filteredGyroX, filteredGyroY, filteredGyroZ, 
                    filteredAccelX, filteredAccelY, filteredAccelZ, 
                    filteredMagX, filteredMagY, filteredMagZ, 
                    dt);
    #else 
        filter.updateIMU(filteredGyroX, filteredGyroY, filteredGyroZ, 
                    filteredAccelX, filteredAccelY, filteredAccelZ,  
                    dt);
    #endif

#elif FILTER_TYPE == FILTER_TYPE_KALMAN

        // Update the EKF filter.
        filter.update(filteredGyroX, filteredGyroY, filteredGyroZ, 
                    filteredAccelX, filteredAccelY, filteredAccelZ, 
                    filteredMagX, filteredMagY, filteredMagZ);

#endif
    }

    lastFilteredAltitude = filteredAltitude;
    climbRate            = 0.0f;

    LOG_INF("Filter warm-up complete.");
}
 
/**
* @brief Updates the IMU sensor data and orientation filter.
*
* Reads raw sensor data (accelerometer and gyroscope), applies calibration offsets,
* orientation transformations, and low-pass filtering. Then, it updates the orientation
* filter and retrieves the current quaternion and Euler angles.
*
* @param dt Time step in seconds.
*/
void ArduFliteIMU::update(float dt) 
{
    if (dt < MIN_DT) dt = MIN_DT;
    if (dt > MAX_DT) dt = MAX_DT;

    // Protect the sensor update with a mutex.
    {
        SemaphoreLock lock(imuMutex);
        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        if (IMU.hasMagnetometer()) 
        {
            IMU.getMag(&magData);
        }

        // For BMP280, update barometer reading.
    #if BARO_TYPE == BARO_TYPE_BMP280
        altitude = bmp280.readAltitude(referencePressure); 
    #endif

        // Remove calibration offsets.
        accelX = accelData.accelX - offsets.accelX;
        accelY = accelData.accelY - offsets.accelY;
        accelZ = accelData.accelZ - offsets.accelZ;
        gyroX  = gyroData.gyroX - offsets.gyroX;
        gyroY  = gyroData.gyroY - offsets.gyroY;
        gyroZ  = gyroData.gyroZ - offsets.gyroZ;

        // Apply sensor orientation adjustments.
        applyOrientation();

        // Update low-pass filters.
        applyLowPassFilters();

        // Compute climb rate (m/s) -----
        // dt is in seconds
        climbRate = (filteredAltitude - lastFilteredAltitude) / dt;
        lastFilteredAltitude = filteredAltitude;

        // Now update your orientation filter with the smoothed values:
    #if FILTER_TYPE == FILTER_TYPE_MADGWICK

        #if IMU_TYPE == IMU_TYPE_MPU9250
        filter.update(filteredGyroX, filteredGyroY, filteredGyroZ, 
                    filteredAccelX, filteredAccelY, filteredAccelZ, 
                    filteredMagX, filteredMagY, filteredMagZ, 
                    dt );
        #else 
        filter.updateIMU(filteredGyroX, filteredGyroY, filteredGyroZ, 
                    filteredAccelX, filteredAccelY, filteredAccelZ,  
                    dt);
        #endif

    #elif FILTER_TYPE == FILTER_TYPE_KALMAN
        // Update the EKF filter.
        filter.update(filteredGyroX, filteredGyroY, filteredGyroZ, 
                    filteredAccelX, filteredAccelY, filteredAccelZ, 
                    filteredMagX, filteredMagY, filteredMagZ);
    #endif

        // Retrieve the computed quaternion and Euler angles.
        filter.getQuaternion(&qw, &qx, &qy, &qz);
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();

        // Update flight state based on the sensor data.
        updateFlightState();
    }

    // Publish to double-buffer for lock-free reads by control loops
    publishSnapshot();
}
 
 /**
  * @brief Applies sensor orientation transformations.
  *
  * Adjusts raw sensor data based on the defined IMU orientation configuration.
  */
 void ArduFliteIMU::applyOrientation() 
 {
 #if (IMU_ORIENTATION == ORIENTATION_NORMAL)
     // No transformation needed.
 #elif (IMU_ORIENTATION == ORIENTATION_SENSOR_FLIPPED_YZ)
     // Transformation: flip gyro X and Z, and invert accelerometer Y.
     gyroX = -gyroX;
     accelY = -accelY;
     gyroZ = -gyroZ;
 #else
     #error "Unknown IMU_ORIENTATION selected!"
 #endif
 }
 
/**
* @brief Performs calibration of the barometer, with the current pressure as the 
* ground-level reference.
*
* Collects raw sensor data over a fixed calibration period, computes average pressure,
* and then saves this. This method assumes the IMU remains stationary during calibration.
*
* @return true if calibration is successful, false otherwise.
*/
bool ArduFliteIMU::baroCalibrate()
{
    LOG_INF("Starting Barometer calibration...");

    const unsigned long CALIB_MS = 1000;
    unsigned long start = millis();
    unsigned int samples = 0;

    double sumBaro = 0;

    // Protect the sensor update with a mutex.
    {
        SemaphoreLock lock(imuMutex);

        while (millis() - start < CALIB_MS) 
        {
            // Read raw sensor data.
    #if BARO_TYPE == BARO_TYPE_BMP280
            double baro = bmp280.readPressure()/100.0; //convert from Pa to hPa
    #else
            double baro = 0.0f;
    #endif

            // Accumulate readings.
            sumBaro += baro;
            samples++;

            delay(5); // Small delay between samples.
        }

        if (samples == 0) 
        {
            LOG_ERR("Baro calibration got no samples!");
            return false;
        }

        double avgBaro      = sumBaro / samples;

        referencePressure    = (float)avgBaro;

        LOG_INF("Pressure reference: %.3f hPa", referencePressure);
    }

    LOG_INF("Barometer calibration done.");

    return true;
}

/**
* @brief Performs self-calibration of the IMU.
*
* Collects raw sensor data over a fixed calibration period, computes average offsets,
* and then saves these offsets to EEPROM. This method assumes the IMU remains stationary
* and level during calibration.
*
* @return true if calibration is successful, false otherwise.
*/
bool ArduFliteIMU::selfCalibrate() 
{
    LOG_INF("=== Self Calibration Start ===");
    LOG_INF("Please keep IMU still & level in final orientation...");

    const unsigned long CALIB_MS = 10000;
    unsigned long start = millis();
    unsigned int samples = 0;

    double sumAx = 0, sumAy = 0, sumAz = 0;
    double sumGx = 0, sumGy = 0, sumGz = 0;

    // Protect the sensor update with a mutex.
    {
        SemaphoreLock lock(imuMutex);

        while (millis() - start < CALIB_MS) 
        {
            // Read raw sensor data.
            IMU.update();
            IMU.getAccel(&accelData);
            IMU.getGyro(&gyroData);

            double ax = accelData.accelX;
            double ay = accelData.accelY;
            double az = accelData.accelZ;
            double gx = gyroData.gyroX;
            double gy = gyroData.gyroY;
            double gz = gyroData.gyroZ;

            // Accumulate readings.
            sumAx   += ax;
            sumAy   += ay;
            sumAz   += az;
            sumGx   += gx;
            sumGy   += gy;
            sumGz   += gz;

            samples++;

            delay(5); // Small delay between samples.
        }

        double avgAx        = sumAx / samples;
        double avgAy        = sumAy / samples;
        double avgAz        = sumAz / samples;
        double avgGx        = sumGx / samples;
        double avgGy        = sumGy / samples;
        double avgGz        = sumGz / samples;

        LOG_INF("Raw average readings:");
        LOG_INF("Accel: %.3f, %.3f, %.3f", avgAx, avgAy, avgAz);
        LOG_INF("Gyro: %.3f, %.3f, %.3f", avgGx, avgGy, avgGz);

        // Determine desired offsets.
        ArduFliteIMUOffsets newOfs;

        newOfs.accelX               = float(avgAx - 0.0);   // Desired X is 0.
        newOfs.accelY               = float(avgAy - 0.0);   // Desired Y is 0.
        newOfs.accelZ               = float(avgAz - 1.0);   // Desired Z is 1.
        newOfs.gyroX                = float(avgGx);         // Desired gyro values are 0.
        newOfs.gyroY                = float(avgGy);
        newOfs.gyroZ                = float(avgGz);

        LOG_INF("Computed new offsets:");
        LOG_INF("Accel Offsets: %.3f, %.3f, %.3f", newOfs.accelX, newOfs.accelY, newOfs.accelZ);
        LOG_INF("Gyro Offsets: %.3f, %.3f, %.3f", newOfs.gyroX, newOfs.gyroY, newOfs.gyroZ);

        // Store and save the calibration offsets.
        setOffsets(newOfs);
        saveOffsetsToEEPROM(newOfs);
    }

    LOG_INF("=== Self Calibration Done ===");

    return true;
}
 
/**
* @brief Applies calibration data from EEPROM.
*
* Loads calibration offsets from EEPROM. If valid data is found, the offsets are applied;
* otherwise, self-calibration is performed.
*
* @return true if calibration offsets are applied successfully, false otherwise.
*/
bool ArduFliteIMU::applyCalibrations() 
{
    ArduFliteIMUOffsets tmp;
    if (loadOffsetsFromEEPROM(tmp)) 
    {
        setOffsets(tmp);
        LOG_INF("Loaded calibration offsets from EEPROM:");
        LOG_INF("Accel: %.3f, %.3f, %.3f", offsets.accelX, offsets.accelY, offsets.accelZ);
        LOG_INF("Gyro:  %.3f, %.3f, %.3f", offsets.gyroX, offsets.gyroY, offsets.gyroZ);

        return true;
    } 
    else 
    {
        LOG_WARN("No valid offsets in EEPROM. Calling selfCalibrate()...");
        return selfCalibrate();
    }
}
 
/**
* @brief Loads calibration offsets from EEPROM.
*
* Retrieves stored calibration data from EEPROM and verifies it using a magic number.
*
* @param dest Reference to an ArduFliteIMUOffsets structure where the offsets will be stored.
* @return true if valid calibration data is found, false otherwise.
*/
bool ArduFliteIMU::loadOffsetsFromEEPROM(ArduFliteIMUOffsets &dest) 
{
    StoredCalibData tmp;
    EEPROM.get(CALIB_DATA_ADDR, tmp);
    if (tmp.magic != CALIB_MAGIC) 
    {
        return false; // No valid data found.
    }
    dest = tmp.offsets;
    return true;
}
 
/**
* @brief Saves calibration offsets to EEPROM.
*
* Stores the provided calibration offsets in EEPROM and commits the changes (ESP32 style).
*
* @param ofs The calibration offsets to save.
*/
void ArduFliteIMU::saveOffsetsToEEPROM(const ArduFliteIMUOffsets &ofs) 
{
    StoredCalibData tmp;
    tmp.magic = CALIB_MAGIC;
    tmp.offsets = ofs;
    EEPROM.put(CALIB_DATA_ADDR, tmp);
    EEPROM.commit();  // Commit changes for ESP32.
    LOG_INF("Calibration data saved to EEPROM.");
}
 
/**
* @brief Sets the calibration offsets.
*
* Updates the internal calibration offsets.
*
* @param ofs The new calibration offsets.
*/
void ArduFliteIMU::setOffsets(const ArduFliteIMUOffsets &ofs) 
{
    offsets = ofs;
}
 
/**
* @brief Retrieves the current calibration offsets.
*
* @param ofs Reference to an ArduFliteIMUOffsets structure where the offsets will be stored.
*/
void ArduFliteIMU::getOffsets(ArduFliteIMUOffsets &ofs) const 
{
    ofs = offsets;
}
 
/**
* @brief Applies low-pass filtering to sensor data.
*
* If the low-pass filters have not been initialized, the filtered sensor values are set
* equal to the raw sensor values. Otherwise, the filters are updated using a simple
* exponential moving average.
*/
void ArduFliteIMU::applyLowPassFilters() 
{
    if (!lpInitialized) 
    {
        filteredAccelX      = accelX;
        filteredAccelY      = accelY;
        filteredAccelZ      = accelZ;
        filteredGyroX       = gyroX;
        filteredGyroY       = gyroY;
        filteredGyroZ       = gyroZ;
        filteredMagX        = magX;
        filteredMagY        = magY;
        filteredMagZ        = magZ;
        filteredAltitude    = altitude;
        lpInitialized       = true;
    } 
    else 
    {
        filteredAccelX = accelAlpha * accelX + (1.0f - accelAlpha) * filteredAccelX;
        filteredAccelY = accelAlpha * accelY + (1.0f - accelAlpha) * filteredAccelY;
        filteredAccelZ = accelAlpha * accelZ + (1.0f - accelAlpha) * filteredAccelZ;

        filteredGyroX  = gyroAlpha * gyroX + (1.0f - gyroAlpha) * filteredGyroX;
        filteredGyroY  = gyroAlpha * gyroY + (1.0f - gyroAlpha) * filteredGyroY;
        filteredGyroZ  = gyroAlpha * gyroZ + (1.0f - gyroAlpha) * filteredGyroZ;
        
        filteredMagX  = magAlpha * magX + (1.0f - magAlpha) * filteredMagX;
        filteredMagY  = magAlpha * magY + (1.0f - magAlpha) * filteredMagY;
        filteredMagZ  = magAlpha * magZ + (1.0f - magAlpha) * filteredMagZ;

        filteredAltitude  = altiAlpha * altitude + (1.0f - altiAlpha) * filteredAltitude;
    }
}

/**
 * @brief Updates the flight state based on current sensor values.
 */
 void ArduFliteIMU::updateFlightState()
 {
    // Calculate the magnitude of the low-pass filtered acceleration.
    float a_mag = sqrt(filteredAccelX * filteredAccelX +
                       filteredAccelY * filteredAccelY +
                       filteredAccelZ * filteredAccelZ);
    // Calculate the magnitude of the low-pass filtered gyro.
    float g_mag = sqrt(filteredGyroX * filteredGyroX +
                       filteredGyroY * filteredGyroY +
                       filteredGyroZ * filteredGyroZ);

     unsigned long now = millis();
 
     // 2) thresholds & timing
     const float ACC_MOV_THR      = 1.0f;   // >g translation
     const float GYRO_THROW_MAX   = 120.0f;  // <deg/s rotation during throw
     const float GYRO_STABLE_THR  = 2.0f;   // <deg/s considered “steady”
     const unsigned long DEBOUNCE  = 50;    // ms of continuous throw-like motion
     const unsigned long STABLE_MS = 2000;  // ms to declare landed
 
     switch (flightState)
     {
       case PREFLIGHT:
         // detect a “throw” = strong accel spike + low rotation
         if ( fabsf(a_mag - 1.0f) >  ACC_MOV_THR && g_mag < GYRO_THROW_MAX )
         {
             if ( now - motionStartTime > DEBOUNCE )
             {
                 flightState = INFLIGHT;
                 flightStableStartTime = now;
             }
         }
         else 
         {
             motionStartTime = now;
         }
         break;
 
       case INFLIGHT:
         // only land if *both* very steady *and* no altitude change
         if ( fabsf(a_mag - 1.0f) < ACC_MOV_THR && g_mag < GYRO_STABLE_THR)
         {
             if ( now - flightStableStartTime >= STABLE_MS )
                 flightState = LANDED;
         }
         else
         {
             flightStableStartTime = now;
         }
         break;
 
       case LANDED:
         // same “throw” test if you pick it up again
         if ( fabsf(a_mag - 1.0f) >  ACC_MOV_THR && g_mag < GYRO_THROW_MAX )
         {
             if ( now - motionStartTime > DEBOUNCE )
             {
                 flightState = INFLIGHT;
                 flightStableStartTime = now;
             }
         }
         else
         {
             motionStartTime = now;
         }
         break;
 
       default:
         flightState = PREFLIGHT;
         break;
     }
 } 

/**
 * @brief Retrieves the current flight state.
 * 
 * Lock-free read from the double-buffer snapshot.
 * 
 * @return The flight state (PREFLIGHT, INFLIGHT, or LANDED).
 */
FlightState ArduFliteIMU::getFlightState() const 
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].flightState;
}
 
/*============================================================================
    Grouped Getters
============================================================================*/
 
/**
* @brief Retrieves the filtered accelerometer data.
*
* Lock-free read from the double-buffer snapshot.
*
* @return Vector3 containing the filtered accelerometer data.
*/
Vector3 ArduFliteIMU::getAcceleration() const 
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].accel;
}
 
/**
* @brief Retrieves the filtered gyroscope data.
*
* Lock-free read from the double-buffer snapshot.
*
* @return Vector3 containing the filtered gyroscope data.
*/
Vector3 ArduFliteIMU::getGyro() const 
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].gyro;
}
 
/**
* @brief Retrieves the magnetometer data.
*
* Lock-free read from the double-buffer snapshot.
*
* @return Vector3 containing the magnetometer data.
*/
Vector3 ArduFliteIMU::getMag() const 
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].mag;
}
 
/**
* @brief Retrieves the current orientation as a quaternion.
*
* Lock-free read from the double-buffer snapshot.
*
* @return FliteQuaternion representing the current orientation.
*/
FliteQuaternion ArduFliteIMU::getQuaternion() const 
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].quat;
}
 
/**
* @brief Retrieves the current orientation as Euler angles.
*
* Lock-free read from the double-buffer snapshot.
*
* @return EulerAngles containing the roll, pitch, and yaw.
*/
EulerAngles ArduFliteIMU::getOrientation() const 
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].orientation;
}

/**
* @brief Retrieves the current estimated altitude in meters.
*
* Lock-free read from the double-buffer snapshot.
*
* @return Altitude in meters.
*/
float ArduFliteIMU::getAltitude() const 
{ 
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].altitude;
}
 
/**
* @brief Retrieves the current estimated climb rate in meters per second.
*
* Lock-free read from the double-buffer snapshot.
*
* @return Climb rate in m/s.
*/
float ArduFliteIMU::getClimbRate() const 
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)].climbRate;
}

// ─────────────────────────────────────────────────────────────────────────────
// Double-buffer implementation for lock-free reads
// ─────────────────────────────────────────────────────────────────────────────

/**
* @brief Publishes current sensor data to the double-buffer.
* 
* Called at the end of update() after all sensor data is processed.
* Writes to the inactive buffer, then atomically swaps the read index.
*/
void ArduFliteIMU::publishSnapshot()
{
    // Write to the buffer that readers are NOT currently using
    int writeIndex = 1 - snapshotReadIndex.load(std::memory_order_relaxed);
    
    ImuSnapshot& snap = snapshotBuffers[writeIndex];
    
    // Fill the snapshot (we already hold imuMutex from update())
    snap.accel.x = filteredAccelX;
    snap.accel.y = filteredAccelY;
    snap.accel.z = filteredAccelZ;
    
    snap.gyro.x = filteredGyroX;
    snap.gyro.y = filteredGyroY;
    snap.gyro.z = filteredGyroZ;
    
    snap.mag.x = magX;
    snap.mag.y = magY;
    snap.mag.z = magZ;
    
    snap.quat.w = qw;
    snap.quat.x = qx;
    snap.quat.y = qy;
    snap.quat.z = qz;
    
    snap.orientation.roll = roll;
    snap.orientation.pitch = pitch;
    snap.orientation.yaw = yaw;
    
    snap.altitude = filteredAltitude;
    snap.climbRate = climbRate;
    snap.flightState = flightState;
    snap.timestampUs = micros();
    
    // Atomic swap: readers will now see the new data
    snapshotReadIndex.store(writeIndex, std::memory_order_release);
}

/**
* @brief Gets a complete lock-free snapshot of all IMU data.
* 
* Returns a copy of the current snapshot buffer. This is the preferred
* method for control loops as it provides consistent data without 
* mutex contention.
*
* @return ImuSnapshot containing all sensor data
*/
ImuSnapshot ArduFliteIMU::getSnapshot() const
{
    return snapshotBuffers[snapshotReadIndex.load(std::memory_order_acquire)];
}