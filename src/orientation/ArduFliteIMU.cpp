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
#include "src/utils/ConfigRegistry.h"
#include "include/ConfigKeys.h"
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

void ArduFliteIMU::initFromConfig()
{
    auto& config = ConfigRegistry::instance();
    
    // Load filter alphas
    accelAlpha = config.get<float>(CONFIG_KEY_IMU_ACCEL_ALPHA);
    gyroAlpha  = config.get<float>(CONFIG_KEY_IMU_GYRO_ALPHA);
    magAlpha   = config.get<float>(CONFIG_KEY_IMU_MAG_ALPHA);
    altiAlpha  = config.get<float>(CONFIG_KEY_IMU_ALTI_ALPHA);
    
    // Madgwick filter tuning
    madgwickBeta = config.get<float>(CONFIG_KEY_IMU_MADGWICK_BETA);
    
    // Load health monitoring thresholds
    maxAccelG     = config.get<float>(CONFIG_KEY_IMU_MAX_ACCEL_G);
    maxGyroDPS    = config.get<float>(CONFIG_KEY_IMU_MAX_GYRO_DPS);
    failThreshold = config.get<uint8_t>(CONFIG_KEY_IMU_FAIL_THRESHOLD);
    
    LOG_INF("ArduFliteIMU: initialized from ConfigRegistry");
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
    // Valid gyro ranges: 250, 500, 1000, 2000 dps. Using 500 for good resolution
    // while still supporting aerobatic maneuvers (0.015 deg/s per LSB).
    err = IMU.setGyroRange(500);
    if (err != 0) 
    {
        LOG_ERR("Error setting gyro range: %d", err);
        return false;
    }
    
    err = IMU.setAccelRange(4);
    if (err != 0) 
    {
        LOG_ERR("Error setting accel range: %d", err);
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
    if (xTaskCreate(imuTask, "IMU Task", 4096, this, 4, &imuTaskHandle) != pdPASS)
    {
        LOG_ERR("Failed to create IMU Task!");
    }
}

/**
 * @brief Suspends the dedicated IMU update task cooperatively.
 *
 * Uses atomic flags to signal the IMU task to pause at a safe point (outside
 * mutex scope), preventing deadlock during calibration. The task spin-waits
 * with vTaskDelay(1ms) until resumeTask() is called - this uses minimal CPU
 * and avoids the complexity of vTaskSuspend/WDT management.
 *
 * @note Caller must call resumeTask() to wake the task.
 */
bool ArduFliteIMU::pauseTask() 
{
    if (imuTaskHandle == NULL) return true;  // No task = already "paused"

    // Signal task to pause at next safe point (after update() releases mutex)
    _pauseRequested.store(true, std::memory_order_release);

    // Wait for task to acknowledge (with 1-second timeout to avoid infinite hang)
    const TickType_t timeout = pdMS_TO_TICKS(1000);
    TickType_t start = xTaskGetTickCount();
    while (!_taskPaused.load(std::memory_order_acquire)) 
    {
        if ((xTaskGetTickCount() - start) > timeout) 
        {
            LOG_ERR("IMU pause timeout - task did not acknowledge!");
            // Clear the request since we're not proceeding
            _pauseRequested.store(false, std::memory_order_release);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Task is now in cooperative spin-wait - no need to vTaskSuspend.
    // Avoiding suspend/resume eliminates WDT re-registration race conditions.
    LOG_INF("IMU Task paused (cooperative spin-wait).");
    return true;
}
 
/**
 * @brief Resumes the dedicated IMU update task.
 *
 * Clears pause flags so the task exits its cooperative spin-wait.
 */
void ArduFliteIMU::resumeTask() 
{
    if (imuTaskHandle == NULL) return;

    // Clear pause flag - task will exit spin-wait on next iteration
    _pauseRequested.store(false, std::memory_order_release);
    
    // Wait briefly for task to acknowledge resume
    vTaskDelay(pdMS_TO_TICKS(5));
    
    LOG_INF("IMU Task resumed.");
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

    // Task period derived from IMU_UPDATE_INTERVAL_MS (single source of truth).
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

        // ─────────────────────────────────────────────────────────────
        // Cooperative pause point - OUTSIDE mutex scope (update() done)
        // ─────────────────────────────────────────────────────────────
        // If pauseTask() was called, acknowledge and spin-wait here until
        // resumeTask() clears the flag. This prevents deadlock because we
        // are NOT holding imuMutex at this point.
        if (imuInstance->_pauseRequested.load(std::memory_order_acquire)) 
        {
            imuInstance->_taskPaused.store(true, std::memory_order_release);
            
            // Spin-wait with yield until resumeTask() clears the request
            // Reset WDT each iteration to prevent timeout during long pauses (e.g., 10s calibration)
            while (imuInstance->_pauseRequested.load(std::memory_order_acquire)) 
            {
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            
            imuInstance->_taskPaused.store(false, std::memory_order_release);
            
            // Reset timing after pause to avoid large dt spike
            lastMicros = micros();
            xLastWakeTime = xTaskGetTickCount();
        }

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
    // Begin the filter at the actual IMU update rate (derived from IMU_UPDATE_INTERVAL_MS).
    filter.begin(IMU_UPDATE_RATE_HZ);
    
    // Set Madgwick beta (gyro/accel trust balance) from config.
    // Higher beta = trust accelerometer more, faster convergence, more noise.
    // Lower beta = trust gyro more, slower convergence, smoother.
#if FILTER_TYPE == FILTER_TYPE_MADGWICK
    filter.setBeta(madgwickBeta);
#endif

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

    // Seed baro data post-warmup to eliminate initial climb rate spike.
    // Without this, filteredAltitude starts at 0 and produces a false spike.
#if BARO_TYPE == BARO_TYPE_BMP280
    LOG_INF("Seeding barometer...");
    for (int i = 0; i < 50; i++) {
        altitude = bmp280.readAltitude(referencePressure);
        filteredAltitude = altiAlpha * altitude + (1.0f - altiAlpha) * filteredAltitude;
        vTaskDelay(pdMS_TO_TICKS(20));  // ~50 Hz baro rate
    }
#endif

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

        // Decimate barometer reads — BMP280 can't produce new data at IMU rate.
        // Track if baro updated this tick for climb rate calculation.
        bool baroUpdated = false;
    #if BARO_TYPE == BARO_TYPE_BMP280
        if (++_baroTickCounter >= BARO_DECIMATION_FACTOR) {
            _baroTickCounter = 0;
            altitude = bmp280.readAltitude(referencePressure);
            baroUpdated = true;
        }
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

        // Validate sensor data for NaN, Inf, and range violations.
        // This updates imuHealthy flag based on consecutive failures.
        validateSensorData();

        // Compute climb rate (m/s) only on baro-update ticks to avoid sawtooth.
        // Use the baro-specific dt (BARO_UPDATE_INTERVAL_MS) for smooth derivative.
    #if BARO_TYPE == BARO_TYPE_BMP280
        if (baroUpdated) {
            constexpr float baroDt = BARO_UPDATE_INTERVAL_MS * 0.001f;
            climbRate = (filteredAltitude - lastFilteredAltitude) / baroDt;
            lastFilteredAltitude = filteredAltitude;
        }
    #else
        // No baro, no climb rate
        climbRate = 0.0f;
    #endif

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
  * All sensor axes (accel, gyro, mag) must be transformed consistently since
  * they share the same physical sensor die orientation.
  */
 void ArduFliteIMU::applyOrientation() 
 {
 #if (IMU_ORIENTATION == ORIENTATION_NORMAL)
     // No transformation needed.
 #elif (IMU_ORIENTATION == ORIENTATION_SENSOR_FLIPPED_YZ)
     // Transformation for sensor mounted with Y and Z axes flipped.
     // Apply same transforms to accel, gyro, and magnetometer.
     // TODO: Verify these transforms match your physical board orientation.
     gyroX = -gyroX;
     accelY = -accelY;
     gyroZ = -gyroZ;
     
     // Magnetometer must also be transformed to match accel/gyro frame
     #if IMU_TYPE == IMU_TYPE_MPU9250
     magY = -magY;
     magZ = -magZ;
     #endif
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

    // ─────────────────────────────────────────────────────────────────────
    // If the IMU task is running, pause it before calibration.
    // This makes selfCalibrate() self-contained and safe to call from anywhere.
    // ─────────────────────────────────────────────────────────────────────
    bool taskWasRunning = (imuTaskHandle != NULL);
    if (taskWasRunning) 
    {
        if (!pauseTask()) 
        {
            LOG_ERR("selfCalibrate: Failed to pause IMU task!");
            return false;
        }
    }

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

            // Yield CPU properly during calibration loop.
            // Task is paused (or never started), so mutex contention is avoided.
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (samples == 0) 
        {
            LOG_ERR("selfCalibrate: No samples collected!");
            if (taskWasRunning) resumeTask();
            return false;
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
        
        // Reset filter state so the low-pass filters reinitialize with new offsets
        lpInitialized = false;
        
        // Reset health monitoring state - give the filter time to converge
        consecutiveFailures = 0;
        imuHealthy.store(true, std::memory_order_release);
    }

    // Resume the IMU task if we paused it
    if (taskWasRunning) 
    {
        resumeTask();
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

/**
 * @brief Checks if the IMU is currently providing valid data.
 * 
 * Thread-safe read of the health status flag.
 *
 * @return true if IMU data is valid and trustworthy, false otherwise.
 */
bool ArduFliteIMU::isHealthy() const
{
    // Lock-free read via atomic bool
    return imuHealthy.load(std::memory_order_acquire);
}

/**
 * @brief Validates sensor readings for NaN, infinity, and range violations.
 * 
 * Checks accelerometer and gyroscope readings against configured limits.
 * Updates consecutiveFailures counter and imuHealthy flag based on results.
 * 
 * @return true if current readings are valid, false otherwise.
 */
bool ArduFliteIMU::validateSensorData()
{
    bool valid = true;
    
    // Check for NaN or infinity in accelerometer data
    if (isnan(filteredAccelX) || isnan(filteredAccelY) || isnan(filteredAccelZ) ||
        isinf(filteredAccelX) || isinf(filteredAccelY) || isinf(filteredAccelZ))
    {
        valid = false;
        LOG_ERR("IMU: Accelerometer NaN/Inf detected!");
    }
    
    // Check for NaN or infinity in gyroscope data
    if (isnan(filteredGyroX) || isnan(filteredGyroY) || isnan(filteredGyroZ) ||
        isinf(filteredGyroX) || isinf(filteredGyroY) || isinf(filteredGyroZ))
    {
        valid = false;
        LOG_ERR("IMU: Gyroscope NaN/Inf detected!");
    }
    
    // Check accelerometer range (values in g)
    if (fabsf(filteredAccelX) > maxAccelG ||
        fabsf(filteredAccelY) > maxAccelG ||
        fabsf(filteredAccelZ) > maxAccelG)
    {
        valid = false;
        LOG_ERR("IMU: Accelerometer out of range! X=%.2f Y=%.2f Z=%.2f", 
                filteredAccelX, filteredAccelY, filteredAccelZ);
    }
    
    // Check gyroscope range (values in deg/s)
    if (fabsf(filteredGyroX) > maxGyroDPS ||
        fabsf(filteredGyroY) > maxGyroDPS ||
        fabsf(filteredGyroZ) > maxGyroDPS)
    {
        valid = false;
        LOG_ERR("IMU: Gyroscope out of range! X=%.2f Y=%.2f Z=%.2f", 
                filteredGyroX, filteredGyroY, filteredGyroZ);
    }
    
    // Update consecutive failure counter and health status.
    // NOTE: No mutex here — caller (update()) already holds imuMutex.
    // imuHealthy is atomic, so isHealthy() readers see consistent values.
    if (valid)
    {
        consecutiveFailures = 0;
        imuHealthy.store(true, std::memory_order_release);
    }
    else
    {
        consecutiveFailures++;
        if (consecutiveFailures >= failThreshold)
        {
            if (imuHealthy.load(std::memory_order_relaxed))
            {
                LOG_ERR("IMU: Marked UNHEALTHY after %u consecutive failures!", 
                        consecutiveFailures);
            }
            imuHealthy.store(false, std::memory_order_release);
        }
    }
    
    return valid;
}

// ─────────────────────────────────────────────────────────────────────────────
// Double-buffer implementation for lock-free reads
// ─────────────────────────────────────────────────────────────────────────────

/**
* @brief Publishes current sensor data to the triple-buffer.
* 
* Called at the end of update() after all sensor data is processed.
* Uses three buffers so a slow reader can never be caught mid-copy by the writer.
*/
void ArduFliteIMU::publishSnapshot()
{
    // Get the pre-determined write index (always different from current read)
    int writeIdx = snapshotWriteIndex.load(std::memory_order_relaxed);
    
    ImuSnapshot& snap = snapshotBuffers[writeIdx];
    
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
    
    // Triple-buffer swap:
    // 1. Get the old read index (might still be in use by a reader)
    int oldReadIdx = snapshotReadIndex.load(std::memory_order_relaxed);
    
    // 2. Publish: make the buffer we just wrote the new read target
    snapshotReadIndex.store(writeIdx, std::memory_order_release);
    
    // 3. Advance write index to the old read buffer (now safe to overwrite)
    //    This ensures we never write to the current read buffer.
    snapshotWriteIndex.store(oldReadIdx, std::memory_order_relaxed);
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