/**
 * ArduFliteIMU.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @file ArduFliteIMU.cpp
 * @brief Implementation of the ArduFliteIMU class.
 *
 * This class interfaces with the IMU sensor (e.g., MPU-6500), handling initialization,
 * calibration, sensor updates, filtering, and orientation estimation. Sensor data is
 * published via a versioned lock-free snapshot (seqlock) for control-loop readers;
 * the IMU task is the sole I2C bus owner and the sole snapshot writer.
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
    imuTaskHandle = nullptr;

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

    snapshotCurrent.quat = FliteQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    snapshotCurrent.flightState = PREFLIGHT;
    snapshotLastComplete = snapshotCurrent;

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

    // Initialize motion debounce timers to the current time so that the first
    // updateMotionSignals() call starts with a fresh debounce window and does
    // not fire a spurious launchDetected at boot if the IMU is already moving.
    motionStartTime       = millis();
    flightStableStartTime = millis();

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
* @brief Warms up the orientation filter and seeds the barometric altitude filter.
*
* Runs 2000 update iterations to settle the orientation filter, then samples the BMP280
* at the baro rate for 50 ticks to seed filteredAltitude so the first update() call
* does not produce a false climb-rate spike.
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
        // Skip bad reads so a NaN/Inf can't poison the seeded filteredAltitude.
        float newAltitude = readBaroAltitude();
        if (isfinite(newAltitude)) {
            altitude = newAltitude;
            filteredAltitude = altiAlpha * altitude + (1.0f - altiAlpha) * filteredAltitude;
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // ~50 Hz baro rate
    }
    _baroFilterInitialized = true;  // filteredAltitude is seeded; update() must not re-seed
#endif // BARO_TYPE == BARO_TYPE_BMP280

    lastFilteredAltitude = filteredAltitude;
    climbRate            = 0.0f;

    LOG_INF("Filter warm-up complete.");
}

#if BARO_TYPE == BARO_TYPE_BMP280
/**
* @brief Computes barometric altitude (m) above the calibrated ground reference.
*
* Uses single-precision powf rather than Adafruit_BMP280::readAltitude(), which
* evaluates the barometric formula in double precision. The ESP32 has a hardware
* float FPU but emulates double in software, so powf is markedly cheaper — this runs
* inside the 500 Hz IMU task's critical section (on baro-decimation ticks).
*
* @return Altitude in meters, or a non-finite value if the pressure read was bad.
*/
float ArduFliteIMU::readBaroAltitude()
{
    float pressurehPa = bmp280.readPressure() / 100.0f;  // Pa -> hPa
    return 44330.0f * (1.0f - powf(pressurehPa / referencePressure, 0.1903f));
}
#endif

/**
* @brief Updates the IMU sensor data and orientation filter.
*
* Runs on the IMU task as the sole owner of the I2C bus. Reads raw accelerometer and
* gyroscope data (and magnetometer if present), reads the barometer decimated to
* BARO_DECIMATION_FACTOR ticks, applies calibration offsets, orientation transformations,
* and low-pass filtering, computes the climb rate on baro-update ticks, validates the
* sensor data, updates the orientation filter, refreshes the motion signals, and
* publishes a coherent lock-free snapshot — all under imuMutex.
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
        if (!lock.acquired())
        {
            // The IMU task is the sole steady-state contender for imuMutex (the
            // calibration paths pause this task before locking), so this should
            // never fire in flight. Guard anyway: a missed acquire must skip the
            // tick — keeping the last good snapshot — rather than driving the I2C
            // bus and writing the snapshot with no lock held. Throttled so a
            // genuine fault stays visible without flooding the console.
            static unsigned long lastWarnMs = 0;
            unsigned long nowMs = millis();
            if (nowMs - lastWarnMs > 1000)
            {
                LOG_WARN("IMU update skipped: could not acquire sensor mutex");
                lastWarnMs = nowMs;
            }
            return;
        }

        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        if (IMU.hasMagnetometer())
        {
            IMU.getMag(&magData);
        }

        // Decimate barometer reads — the BMP280 produces new data far slower than the
        // 500 Hz IMU rate. Read, low-pass, and differentiate the altitude here at the
        // baro rate (~50 Hz). Filtering at the baro rate (rather than every IMU tick)
        // keeps altiAlpha's cutoff matched to the actual sample rate.
    #if BARO_TYPE == BARO_TYPE_BMP280
        if (++_baroTickCounter >= BARO_DECIMATION_FACTOR)
        {
            _baroTickCounter = 0;

            // A single NaN/Inf would permanently poison the altitude EMA (and hence
            // climbRate and the published snapshot), since NaN propagates through every
            // subsequent EMA term. Keep the last good altitude on a bad read.
            float newAltitude = readBaroAltitude();
            if (isfinite(newAltitude))
            {
                altitude = newAltitude;

                if (!_baroFilterInitialized)
                {
                    // First sample after boot or recalibration: seed without a
                    // derivative so there is no false climb-rate spike.
                    filteredAltitude       = altitude;
                    lastFilteredAltitude   = altitude;
                    climbRate              = 0.0f;
                    _baroFilterInitialized = true;
                }
                else
                {
                    filteredAltitude = altiAlpha * altitude + (1.0f - altiAlpha) * filteredAltitude;
                    constexpr float baroDt = BARO_UPDATE_INTERVAL_MS * 0.001f;
                    climbRate = (filteredAltitude - lastFilteredAltitude) / baroDt;
                    lastFilteredAltitude = filteredAltitude;
                }
            }
        }
    #else
        // No barometer: no altitude or climb rate.
        climbRate = 0.0f;
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

        // Update orientation filter with filtered sensor values.
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

        // Update motion signals (throw/stable detection) based on the sensor data.
        updateMotionSignals();

        // Publish a consistent lock-free snapshot before releasing imuMutex.
        publishSnapshot();
    }
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

    float sumBaro = 0.0f;

    // Acquire and release imuMutex per sample — never hold across vTaskDelay.
    // Matches selfCalibrate() pattern; prevents IMU-task deadlock if called post-start.
    while (millis() - start < CALIB_MS)
    {
        {
            SemaphoreLock lock(imuMutex);
            if (!lock.acquired())
            {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
    #if BARO_TYPE == BARO_TYPE_BMP280
            float baro = bmp280.readPressure() / 100.0f; // Pa → hPa
    #else
            float baro = 0.0f;
    #endif
            sumBaro += baro;
            samples++;
        } // release mutex before yielding

        vTaskDelay(pdMS_TO_TICKS(5)); // FreeRTOS-aware yield between samples.
    }

    if (samples == 0)
    {
        LOG_ERR("Baro calibration got no samples!");
        return false;
    }

    {
        SemaphoreLock lock(imuMutex);
        if (!lock.acquired())
        {
            LOG_ERR("Baro calibration could not acquire sensor mutex for final update");
            return false;
        }
        referencePressure = sumBaro / static_cast<float>(samples);
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

    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;

    // Protect the sensor update with a mutex, but yield (vTaskDelay) OUTSIDE
    // the lock so other tasks can acquire imuMutex between samples.
    while (millis() - start < CALIB_MS)
    {
        float ax, ay, az, gx, gy, gz;
        {
            SemaphoreLock lock(imuMutex);
            if (!lock.acquired())
            {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            IMU.update();
            IMU.getAccel(&accelData);
            IMU.getGyro(&gyroData);
            ax = accelData.accelX;
            ay = accelData.accelY;
            az = accelData.accelZ;
            gx = gyroData.gyroX;
            gy = gyroData.gyroY;
            gz = gyroData.gyroZ;
        }
        sumAx += ax; sumAy += ay; sumAz += az;
        sumGx += gx; sumGy += gy; sumGz += gz;
        samples++;
        // Yield CPU between samples without holding the mutex.
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (samples == 0)
    {
        LOG_ERR("selfCalibrate: No samples collected!");
        if (taskWasRunning) resumeTask();
        return false;
    }

    const float avgAx = sumAx / samples;
    const float avgAy = sumAy / samples;
    const float avgAz = sumAz / samples;
    const float avgGx = sumGx / samples;
    const float avgGy = sumGy / samples;
    const float avgGz = sumGz / samples;

    LOG_INF("Raw average readings:");
    LOG_INF("Accel: %.3f, %.3f, %.3f", avgAx, avgAy, avgAz);
    LOG_INF("Gyro: %.3f, %.3f, %.3f", avgGx, avgGy, avgGz);

    ArduFliteIMUOffsets newOfs;
    newOfs.accelX = avgAx - 0.0f;
    newOfs.accelY = avgAy - 0.0f;
    newOfs.accelZ = avgAz - 1.0f;
    newOfs.gyroX  = avgGx;
    newOfs.gyroY  = avgGy;
    newOfs.gyroZ  = avgGz;

    LOG_INF("Computed new offsets:");
    LOG_INF("Accel Offsets: %.3f, %.3f, %.3f", newOfs.accelX, newOfs.accelY, newOfs.accelZ);
    LOG_INF("Gyro Offsets: %.3f, %.3f, %.3f", newOfs.gyroX, newOfs.gyroY, newOfs.gyroZ);

    {
        SemaphoreLock lock(imuMutex);
        if (!lock.acquired())
        {
            LOG_ERR("selfCalibrate: could not acquire sensor mutex for offset update");
            if (taskWasRunning) resumeTask();
            return false;
        }
        setOffsets(newOfs);
        lpInitialized       = false;
    #if BARO_TYPE == BARO_TYPE_BMP280
        // Re-seed the altitude filter on the next baro tick so the climb rate does not
        // spike across the calibration pause (filteredAltitude/lastFilteredAltitude
        // would otherwise be differentiated against a stale pre-pause sample).
        _baroFilterInitialized = false;
        _baroTickCounter       = 0;
    #endif
        consecutiveFailures = 0;
        imuHealthy.store(true, std::memory_order_release);
    }
    saveOffsetsToEEPROM(newOfs);

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

    }
}

/**
 * @brief Computes debounced motion signals from filtered sensor data.
 *
 * The IMU  only produces launchDetected / stableDetected boolean signals.
 * FlightState transitions are the sole responsibility of StateManagement,
 * which reads these signals each loop tick via getMotionSignals().
 *
 * Both signals run independent debounce timers so StateManagement can apply
 * whatever transition logic it needs without the IMU knowing about FlightState.
 */
void ArduFliteIMU::updateMotionSignals()
{
    // Compute filtered vector magnitudes.
    // Both accel and gyro now use squared magnitudes — sqrtf avoided entirely.
    // |a_mag - 1| > THR  ⟺  a_magSq > (1+THR)²  or  a_magSq < (1-THR)²
    // |a_mag - 1| < THR  ⟺  (1-THR)² ≤ a_magSq ≤ (1+THR)²
    // g_magSq uses the squared magnitude to avoid a sqrtf at 500 Hz —
    // all threshold comparisons are squared accordingly.
    float a_magSq = filteredAccelX * filteredAccelX +
                    filteredAccelY * filteredAccelY +
                    filteredAccelZ * filteredAccelZ;
    float g_magSq = filteredGyroX * filteredGyroX +
                    filteredGyroY * filteredGyroY +
                    filteredGyroZ * filteredGyroZ;

    unsigned long now = millis();

    // Detection thresholds.
    constexpr float         ACC_THROW_THR      = 0.10f;  ///< g deviation above gravity to detect a throw (accel branch)
    constexpr float         ACC_STABLE_THR     = 0.30f;  ///< g deviation within which the aircraft is considered stable after landing
    constexpr float         GYRO_THROW_MIN     = 15.0f;  ///< deg/s minimum rotation to qualify as a throw (soft-launch trigger)
    constexpr float         GYRO_THROW_MAX     = 150.0f; ///< deg/s must be below this during a throw
    constexpr float         GYRO_STABLE_THR    = 2.0f;   ///< deg/s below this is considered "steady"
    constexpr float         GYRO_THROW_MIN_SQ  = GYRO_THROW_MIN  * GYRO_THROW_MIN;
    constexpr float         GYRO_THROW_MAX_SQ  = GYRO_THROW_MAX  * GYRO_THROW_MAX;
    constexpr float         GYRO_STABLE_THR_SQ = GYRO_STABLE_THR * GYRO_STABLE_THR;
    // Squared accel thresholds: |a-1|>T  ⟺  a_magSq>(1+T)² or a_magSq<(1-T)²
    constexpr float         ACC_THROW_HI_SQ    = (1.0f + ACC_THROW_THR)  * (1.0f + ACC_THROW_THR);
    constexpr float         ACC_THROW_LO_SQ    = (1.0f - ACC_THROW_THR)  * (1.0f - ACC_THROW_THR);
    constexpr float         ACC_STABLE_HI_SQ   = (1.0f + ACC_STABLE_THR) * (1.0f + ACC_STABLE_THR);
    constexpr float         ACC_STABLE_LO_SQ   = (1.0f - ACC_STABLE_THR) * (1.0f - ACC_STABLE_THR);
    constexpr unsigned long DEBOUNCE           = 50;     ///< ms motion must sustain to trigger launchDetected
    constexpr unsigned long STABLE_MS          = 2000;   ///< ms stability must sustain to trigger stableDetected

    // Throw detection: combined accel OR gyro trigger, gyro within flight range.
    // The OR condition supports soft hand-launches where accel deviation is small
    // but rotation (e.g. gyro_y) clearly exceeds GYRO_THROW_MIN.
    const bool accelThrow = (a_magSq > ACC_THROW_HI_SQ || a_magSq < ACC_THROW_LO_SQ);
    if ((accelThrow || g_magSq > GYRO_THROW_MIN_SQ) && g_magSq < GYRO_THROW_MAX_SQ)
    {
        _launchDetected = (now - motionStartTime > DEBOUNCE);
    }
    else
    {
        motionStartTime = now;  // Reset timer while condition is not met
        _launchDetected  = false;
    }

    // Stability detection: sustained near-1g, low-rotation condition (on the ground).
    // ACC_STABLE_THR is intentionally looser than ACC_THROW_THR to tolerate rough
    // terrain, grass, or slight inclines without blocking the LANDED transition.
    const bool accelStable = (a_magSq >= ACC_STABLE_LO_SQ && a_magSq <= ACC_STABLE_HI_SQ);
    if (accelStable && g_magSq < GYRO_STABLE_THR_SQ)
    {
        _stableDetected = (now - flightStableStartTime >= STABLE_MS);
    }
    else
    {
        flightStableStartTime = now;  // Reset timer while condition is not met
        _stableDetected       = false;
    }
}

/**
 * @brief Updates the display flight state stored in the snapshot.
 *
 * Called by StateManagement after each FlightState transition so that
 * telemetry and the CLI stream command continue to show the correct state.
 * Thread-safe: writes to an internal atomic variable picked up by publishSnapshot().
 *
 * @param state New FlightState to report.
 */
void ArduFliteIMU::setFlightState(FlightState state)
{
    _flightState.store(static_cast<int>(state), std::memory_order_release);
}

/**
 * @brief Returns the latest debounced motion signals.
 *
 * Lock-free read from the versioned snapshot. Called by StateManagement
 * each loop tick to decide FlightState transitions.
 *
 * @return MotionSignals containing launchDetected and stableDetected.
 */
MotionSignals ArduFliteIMU::getMotionSignals() const
{
    return getSnapshot().motion;
}

/**
 * @brief Retrieves the current flight state.
 *
 * Lock-free read from the versioned snapshot.
 *
 * @return The flight state (PREFLIGHT, INFLIGHT, or LANDED).
 */
FlightState ArduFliteIMU::getFlightState() const
{
    return getSnapshot().flightState;
}

/*============================================================================
    Grouped Getters
============================================================================*/

/**
* @brief Retrieves the filtered accelerometer data.
*
* Lock-free read from the versioned snapshot.
*
* @return Vector3 containing the filtered accelerometer data.
*/
Vector3 ArduFliteIMU::getAcceleration() const
{
    return getSnapshot().accel;
}

/**
* @brief Retrieves the filtered gyroscope data.
*
* Lock-free read from the versioned snapshot.
*
* @return Vector3 containing the filtered gyroscope data.
*/
Vector3 ArduFliteIMU::getGyro() const
{
    return getSnapshot().gyro;
}

/**
* @brief Retrieves the magnetometer data.
*
* Lock-free read from the versioned snapshot.
*
* @return Vector3 containing the magnetometer data.
*/
Vector3 ArduFliteIMU::getMag() const
{
    return getSnapshot().mag;
}

/**
* @brief Retrieves the current orientation as a quaternion.
*
* Lock-free read from the versioned snapshot.
*
* @return FliteQuaternion representing the current orientation.
*/
FliteQuaternion ArduFliteIMU::getQuaternion() const
{
    return getSnapshot().quat;
}

/**
* @brief Retrieves the current orientation as Euler angles.
*
* Lock-free read from the versioned snapshot.
*
* @return EulerAngles containing the roll, pitch, and yaw.
*/
EulerAngles ArduFliteIMU::getOrientation() const
{
    return getSnapshot().orientation;
}

/**
* @brief Retrieves the current estimated altitude in meters.
*
* Lock-free read from the versioned snapshot.
*
* @return Altitude in meters.
*/
float ArduFliteIMU::getAltitude() const
{
    return getSnapshot().altitude;
}

/**
* @brief Retrieves the current estimated climb rate in meters per second.
*
* Lock-free read from the versioned snapshot.
*
* @return Climb rate in m/s.
*/
float ArduFliteIMU::getClimbRate() const
{
    return getSnapshot().climbRate;
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
    static unsigned long lastFaultLogMs = 0;
    unsigned long nowMs = millis();
    bool logFaultDetails = (nowMs - lastFaultLogMs > 1000);
    if (logFaultDetails) lastFaultLogMs = nowMs;

    // Check for NaN or infinity in accelerometer data
    if (isnan(filteredAccelX) || isnan(filteredAccelY) || isnan(filteredAccelZ) ||
        isinf(filteredAccelX) || isinf(filteredAccelY) || isinf(filteredAccelZ))
    {
        valid = false;
        if (logFaultDetails) LOG_ERR("IMU: Accelerometer NaN/Inf detected!");
    }

    // Check for NaN or infinity in gyroscope data
    if (isnan(filteredGyroX) || isnan(filteredGyroY) || isnan(filteredGyroZ) ||
        isinf(filteredGyroX) || isinf(filteredGyroY) || isinf(filteredGyroZ))
    {
        valid = false;
        if (logFaultDetails) LOG_ERR("IMU: Gyroscope NaN/Inf detected!");
    }

    // Check accelerometer range (values in g)
    if (fabsf(filteredAccelX) > maxAccelG ||
        fabsf(filteredAccelY) > maxAccelG ||
        fabsf(filteredAccelZ) > maxAccelG)
    {
        valid = false;
        if (logFaultDetails)
        {
            LOG_ERR("IMU: Accelerometer out of range! X=%.2f Y=%.2f Z=%.2f",
                    filteredAccelX, filteredAccelY, filteredAccelZ);
        }
    }

    // Check gyroscope range (values in deg/s)
    if (fabsf(filteredGyroX) > maxGyroDPS ||
        fabsf(filteredGyroY) > maxGyroDPS ||
        fabsf(filteredGyroZ) > maxGyroDPS)
    {
        valid = false;
        if (logFaultDetails)
        {
            LOG_ERR("IMU: Gyroscope out of range! X=%.2f Y=%.2f Z=%.2f",
                    filteredGyroX, filteredGyroY, filteredGyroZ);
        }
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
// Versioned snapshot implementation for lock-free reads
// ─────────────────────────────────────────────────────────────────────────────

/**
* @brief Publishes current sensor data to the versioned snapshot.
*
* Called at the end of update() after all sensor data is processed.
* Uses an odd/even version counter so readers can retry torn copies.
*/
void ArduFliteIMU::publishSnapshot()
{
    publishLastCompleteSnapshot();

    uint32_t version = snapshotVersion.load(std::memory_order_relaxed);
    snapshotVersion.store(version + 1, std::memory_order_release);

    snapshotCurrent.accel.x = filteredAccelX;
    snapshotCurrent.accel.y = filteredAccelY;
    snapshotCurrent.accel.z = filteredAccelZ;

    snapshotCurrent.gyro.x = filteredGyroX;
    snapshotCurrent.gyro.y = filteredGyroY;
    snapshotCurrent.gyro.z = filteredGyroZ;

    snapshotCurrent.mag.x = magX;
    snapshotCurrent.mag.y = magY;
    snapshotCurrent.mag.z = magZ;

    snapshotCurrent.quat.w = qw;
    snapshotCurrent.quat.x = qx;
    snapshotCurrent.quat.y = qy;
    snapshotCurrent.quat.z = qz;

    snapshotCurrent.orientation.roll = roll;
    snapshotCurrent.orientation.pitch = pitch;
    snapshotCurrent.orientation.yaw = yaw;

    snapshotCurrent.altitude = filteredAltitude;
    snapshotCurrent.climbRate = climbRate;
    snapshotCurrent.flightState = static_cast<FlightState>(_flightState.load(std::memory_order_acquire));
    snapshotCurrent.motion.launchDetected = _launchDetected;
    snapshotCurrent.motion.stableDetected = _stableDetected;
    snapshotCurrent.timestampUs = micros();

    snapshotVersion.store(version + 2, std::memory_order_release);
}

void ArduFliteIMU::publishLastCompleteSnapshot()
{
    uint32_t version = snapshotLastCompleteVersion.load(std::memory_order_relaxed);
    snapshotLastCompleteVersion.store(version + 1, std::memory_order_release);
    snapshotLastComplete = snapshotCurrent;
    snapshotLastCompleteVersion.store(version + 2, std::memory_order_release);
}

/**
* @brief Gets a complete lock-free snapshot of all IMU data.
*
* Returns a copy of the current snapshot. This is the preferred
* method for control loops as it provides consistent data without
* mutex contention.
*
* @return ImuSnapshot containing all sensor data
*/
ImuSnapshot ArduFliteIMU::getSnapshot() const
{
    ImuSnapshot snap;
    uint32_t retries = 0;

    for (;;)
    {
        uint32_t before = snapshotVersion.load(std::memory_order_acquire);
        if ((before & 1U) != 0U)
        {
            ++retries;
        }
        else
        {
            snap = snapshotCurrent;

            uint32_t after = snapshotVersion.load(std::memory_order_acquire);
            if (before == after)
            {
                recordSnapshotReadRetries(retries, false);
                return snap;
            }

            ++retries;
        }

        if (retries >= SNAPSHOT_READ_RETRY_LIMIT)
        {
            recordSnapshotReadRetries(retries, true);
            return getLastCompleteSnapshot();
        }
    }
}

ImuSnapshot ArduFliteIMU::getLastCompleteSnapshot() const
{
    ImuSnapshot snap;

    for (;;)
    {
        uint32_t before = snapshotLastCompleteVersion.load(std::memory_order_acquire);
        if ((before & 1U) != 0U) continue;

        snap = snapshotLastComplete;

        uint32_t after = snapshotLastCompleteVersion.load(std::memory_order_acquire);
        if (before == after) return snap;
    }
}

void ArduFliteIMU::recordSnapshotReadRetries(uint32_t retries, bool limitHit) const
{
    if (retries > 0)
    {
        snapshotTotalReadRetries.fetch_add(retries, std::memory_order_relaxed);

        uint32_t maxRetries = snapshotMaxReadRetries.load(std::memory_order_relaxed);
        while (retries > maxRetries &&
               !snapshotMaxReadRetries.compare_exchange_weak(
                   maxRetries,
                   retries,
                   std::memory_order_relaxed,
                   std::memory_order_relaxed))
        {
        }
    }

    if (limitHit)
    {
        snapshotRetryLimitHits.fetch_add(1, std::memory_order_relaxed);
    }
}

ImuSnapshotHealth ArduFliteIMU::getSnapshotHealth() const
{
    return {
        snapshotTotalReadRetries.load(std::memory_order_relaxed),
        snapshotMaxReadRetries.load(std::memory_order_relaxed),
        snapshotRetryLimitHits.load(std::memory_order_relaxed),
    };
}
