/**
 * @file ArduFliteIMU.cpp
 * @brief Implementation of the ArduFliteIMU class.
 *
 * This class interfaces with the IMU sensor (e.g., MPU-6500), handling initialization,
 * calibration, sensor updates, filtering, and orientation estimation. Shared sensor data
 * is protected by a FreeRTOS mutex.
 */

 #include "src/orientation/ArduFliteIMU.h"
 #include <math.h>

 /**
  * @brief Constructor.
  *
  * Initializes calibration offsets to zero and creates a mutex to protect the IMU data.
  */
 ArduFliteIMU::ArduFliteIMU() {
     // Initialize calibration offsets to zero.
     offsets = {0, 0, 0, 0, 0, 0};
 
     // Create the mutex for protecting sensor data.
     imuMutex = xSemaphoreCreateMutex();
     if (imuMutex == NULL) {
         Serial.println("Failed to create IMU mutex!");
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
 bool ArduFliteIMU::begin() {
     // For ESP32, initialize EEPROM.
     EEPROM.begin(EEPROM_SIZE);
     Wire.begin();
     Wire.setClock(400000); // Set I2C clock to 400 kHz.
 
     // Initialize IMU with calibration data.
     int err = IMU.init(calib, IMU_ADDRESS);
     if (err != 0) {
         Serial.print("FastIMU init error: ");
         Serial.println(err);
         return false;
     }
 
     // Set sensor ranges.
     err = IMU.setGyroRange(500);
     err = IMU.setAccelRange(4);
 
     if (err == -1) {
         Serial.print("Error setting IMU sensor ranges: ");
         Serial.println(err);
         return false;
     }
 
     // Load calibration offsets from EEPROM; if unavailable, perform self-calibration.
     if (!applyCalibrations()) {
        Serial.println("FastIMU failed to apply calibrations!");
        return false;
     }
 
     // Initialize and warm up the orientation filter.
     initFilter();
 
     Serial.println("FastIMU (MPU-6500) initialized!");
     return true;
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
     for (int i = 0; i < 2000; i++) {
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
         // Update the orientation filter using the filtered sensor values.
         filter.update(filteredGyroX, filteredGyroY, filteredGyroZ, 
                       filteredAccelX, filteredAccelY, filteredAccelZ, 
                       filteredMagX, filteredMagY, filteredMagZ, 
                       dt);
 #elif FILTER_TYPE == FILTER_TYPE_KALMAN
         // Update the EKF filter.
         filter.update(filteredGyroX, filteredGyroY, filteredGyroZ, 
                       filteredAccelX, filteredAccelY, filteredAccelZ, 
                       filteredMagX, filteredMagY, filteredMagZ);
 #endif
     }
     Serial.println("Filter warm-up complete.");
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
     xSemaphoreTake(imuMutex, portMAX_DELAY);
 
     IMU.update();
     IMU.getAccel(&accelData);
     IMU.getGyro(&gyroData);
 
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
 
 #if FILTER_TYPE == FILTER_TYPE_MADGWICK
    // Now update your orientation filter with the smoothed values:
    //TODO: use this block when we have a magnetometer!
    // filter.update(filteredGyroX, filteredGyroY, filteredGyroZ, 
    //     filteredAccelX, filteredAccelY, filteredAccelZ, 
    //     filteredMagX, filteredMagY, filteredMagZ, 
    //     dt
    // );
     filter.updateIMU(filteredGyroX, filteredGyroY, filteredGyroZ, 
                      filteredAccelX, filteredAccelY, filteredAccelZ,  
                      dt);
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
 
     xSemaphoreGive(imuMutex);
 }
 
 /**
  * @brief Applies sensor orientation transformations.
  *
  * Adjusts raw sensor data based on the defined IMU orientation configuration.
  */
 void ArduFliteIMU::applyOrientation() {
 #if (IMU_ORIENTATION == ORIENTATION_NORMAL)
     // No transformation needed.
 #elif (IMU_ORIENTATION == ORIENTATION_SENSOR_FLIPPED_YZ)
     // Example transformation: flip gyro X and Z, and invert accelerometer Y.
     gyroX = -gyroX;
     accelY = -accelY;
     gyroZ = -gyroZ;
 #else
     #error "Unknown IMU_ORIENTATION selected!"
 #endif
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
 bool ArduFliteIMU::selfCalibrate() {
     Serial.println("=== Self Calibration Start ===");
     Serial.println("Please keep IMU still & level in final orientation...");
 
     const unsigned long CALIB_MS = 10000;
     unsigned long start = millis();
     unsigned int samples = 0;
 
     double sumAx = 0, sumAy = 0, sumAz = 0;
     double sumGx = 0, sumGy = 0, sumGz = 0;

    // Protect the sensor update with a mutex.
    xSemaphoreTake(imuMutex, portMAX_DELAY);
 
    while (millis() - start < CALIB_MS) {
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
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        samples++;

        delay(5); // Small delay between samples.
    }

    double avgAx = sumAx / samples;
    double avgAy = sumAy / samples;
    double avgAz = sumAz / samples;
    double avgGx = sumGx / samples;
    double avgGy = sumGy / samples;
    double avgGz = sumGz / samples;

    Serial.println("Raw average readings:");
    Serial.print("Accel: ");
    Serial.print(avgAx, 3); Serial.print(", ");
    Serial.print(avgAy, 3); Serial.print(", ");
    Serial.println(avgAz, 3);

    Serial.print("Gyro: ");
    Serial.print(avgGx, 3); Serial.print(", ");
    Serial.print(avgGy, 3); Serial.print(", ");
    Serial.println(avgGz, 3);

    // Determine desired offsets.
    ArduFliteIMUOffsets newOfs;
    newOfs.accelX = float(avgAx - 0.0);   // Desired X is 0.
    newOfs.accelY = float(avgAy - 0.0);   // Desired Y is 0.
    newOfs.accelZ = float(avgAz - 1.0);   // Desired Z is 1.
    newOfs.gyroX  = float(avgGx);         // Desired gyro values are 0.
    newOfs.gyroY  = float(avgGy);
    newOfs.gyroZ  = float(avgGz);

    Serial.println("Computed new offsets:");
    Serial.print("Accel Offsets: ");
    Serial.print(newOfs.accelX, 3); Serial.print(", ");
    Serial.print(newOfs.accelY, 3); Serial.print(", ");
    Serial.println(newOfs.accelZ, 3);

    Serial.print("Gyro Offsets: ");
    Serial.print(newOfs.gyroX, 3); Serial.print(", ");
    Serial.print(newOfs.gyroY, 3); Serial.print(", ");
    Serial.println(newOfs.gyroZ, 3);

    // Store and save the calibration offsets.
    setOffsets(newOfs);
    saveOffsetsToEEPROM(newOfs);

    Serial.println("=== Self Calibration Done ===");

    xSemaphoreGive(imuMutex);

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
 bool ArduFliteIMU::applyCalibrations() {
     ArduFliteIMUOffsets tmp;
     if (loadOffsetsFromEEPROM(tmp)) {
         setOffsets(tmp);
         Serial.println("Loaded calibration offsets from EEPROM:");
         Serial.print("Accel: ");
         Serial.print(offsets.accelX, 3); Serial.print(", ");
         Serial.print(offsets.accelY, 3); Serial.print(", ");
         Serial.println(offsets.accelZ, 3);
 
         Serial.print("Gyro: ");
         Serial.print(offsets.gyroX, 3); Serial.print(", ");
         Serial.print(offsets.gyroY, 3); Serial.print(", ");
         Serial.println(offsets.gyroZ, 3);
         return true;
     } else {
         Serial.println("No valid offsets in EEPROM. Consider calling selfCalibrate().");
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
 bool ArduFliteIMU::loadOffsetsFromEEPROM(ArduFliteIMUOffsets &dest) {
     StoredCalibData tmp;
     EEPROM.get(CALIB_DATA_ADDR, tmp);
     if (tmp.magic != CALIB_MAGIC) {
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
 void ArduFliteIMU::saveOffsetsToEEPROM(const ArduFliteIMUOffsets &ofs) {
     StoredCalibData tmp;
     tmp.magic = CALIB_MAGIC;
     tmp.offsets = ofs;
     EEPROM.put(CALIB_DATA_ADDR, tmp);
     EEPROM.commit();  // Commit changes for ESP32.
     Serial.println("Calibration data saved to EEPROM.");
 }
 
 /**
  * @brief Sets the calibration offsets.
  *
  * Updates the internal calibration offsets.
  *
  * @param ofs The new calibration offsets.
  */
 void ArduFliteIMU::setOffsets(const ArduFliteIMUOffsets &ofs) {
     offsets = ofs;
 }
 
 /**
  * @brief Retrieves the current calibration offsets.
  *
  * @param ofs Reference to an ArduFliteIMUOffsets structure where the offsets will be stored.
  */
 void ArduFliteIMU::getOffsets(ArduFliteIMUOffsets &ofs) const {
     ofs = offsets;
 }
 
 /**
  * @brief Applies low-pass filtering to sensor data.
  *
  * If the low-pass filters have not been initialized, the filtered sensor values are set
  * equal to the raw sensor values. Otherwise, the filters are updated using a simple
  * exponential moving average.
  */
 void ArduFliteIMU::applyLowPassFilters() {
     if (!lpInitialized) {
         filteredAccelX = accelX;
         filteredAccelY = accelY;
         filteredAccelZ = accelZ;
         filteredGyroX  = gyroX;
         filteredGyroY  = gyroY;
         filteredGyroZ  = gyroZ;
         lpInitialized = true;
     } else {
         filteredAccelX = accelAlpha * accelX + (1.0f - accelAlpha) * filteredAccelX;
         filteredAccelY = accelAlpha * accelY + (1.0f - accelAlpha) * filteredAccelY;
         filteredAccelZ = accelAlpha * accelZ + (1.0f - accelAlpha) * filteredAccelZ;
 
         filteredGyroX  = gyroAlpha * gyroX + (1.0f - gyroAlpha) * filteredGyroX;
         filteredGyroY  = gyroAlpha * gyroY + (1.0f - gyroAlpha) * filteredGyroY;
         filteredGyroZ  = gyroAlpha * gyroZ + (1.0f - gyroAlpha) * filteredGyroZ;
     }
 }
 
 /*============================================================================
   Grouped Getters
   ============================================================================*/
 
 /**
  * @brief Retrieves the filtered accelerometer data.
  *
  * Uses the mutex to ensure a consistent snapshot of the sensor values.
  *
  * @return Vector3 containing the filtered accelerometer data.
  */
 Vector3 ArduFliteIMU::getAcceleration() const {
     Vector3 acc;
     xSemaphoreTake(imuMutex, portMAX_DELAY);
     acc.x = filteredAccelX;
     acc.y = filteredAccelY;
     acc.z = filteredAccelZ;
     xSemaphoreGive(imuMutex);
     return acc;
 }
 
 /**
  * @brief Retrieves the filtered gyroscope data.
  *
  * Uses the mutex to ensure thread-safe access.
  *
  * @return Vector3 containing the filtered gyroscope data.
  */
 Vector3 ArduFliteIMU::getGyro() const {
     Vector3 gyro;
     xSemaphoreTake(imuMutex, portMAX_DELAY);
     gyro.x = filteredGyroX;
     gyro.y = filteredGyroY;
     gyro.z = filteredGyroZ;
     xSemaphoreGive(imuMutex);
     return gyro;
 }
 
 /**
  * @brief Retrieves the magnetometer data.
  *
  * Uses the mutex to provide a consistent snapshot.
  *
  * @return Vector3 containing the magnetometer data.
  */
 Vector3 ArduFliteIMU::getMag() const {
     Vector3 mag;
     xSemaphoreTake(imuMutex, portMAX_DELAY);
     mag.x = magX;
     mag.y = magY;
     mag.z = magZ;
     xSemaphoreGive(imuMutex);
     return mag;
 }
 
 /**
  * @brief Retrieves the current orientation as a quaternion.
  *
  * Uses the mutex to ensure that the quaternion is read atomically.
  *
  * @return FliteQuaternion representing the current orientation.
  */
 FliteQuaternion ArduFliteIMU::getQuaternion() const {
     FliteQuaternion q;
     xSemaphoreTake(imuMutex, portMAX_DELAY);
     q.w = qw;
     q.x = qx;
     q.y = qy;
     q.z = qz;
     xSemaphoreGive(imuMutex);
     return q;
 }
 
 /**
  * @brief Retrieves the current orientation as Euler angles.
  *
  * Uses the mutex to safely read roll, pitch, and yaw.
  *
  * @return EulerAngles containing the roll, pitch, and yaw.
  */
 EulerAngles ArduFliteIMU::getOrientation() const {
     EulerAngles ang;
     xSemaphoreTake(imuMutex, portMAX_DELAY);
     ang.roll  = roll;
     ang.pitch = pitch;
     ang.yaw   = yaw;
     xSemaphoreGive(imuMutex);
     return ang;
 }
 
 /**
 * @brief Updates the flight state based on current sensor values.
 */
 void ArduFliteIMU::updateFlightState() {
    // Calculate the magnitude of the low-pass filtered acceleration.
    float a_mag = sqrt(filteredAccelX * filteredAccelX +
                       filteredAccelY * filteredAccelY +
                       filteredAccelZ * filteredAccelZ);
    // Calculate the magnitude of the low-pass filtered gyro.
    float g_mag = sqrt(filteredGyroX * filteredGyroX +
                       filteredGyroY * filteredGyroY +
                       filteredGyroZ * filteredGyroZ);

    unsigned long now = millis();

    // Define thresholds (adjust these values based on testing/calibration).
    const float ACC_THRESHOLD = 0.1f;     // Tolerance around 1g.
    const float GYRO_THRESHOLD = 5.0f;      // Maximum gyro rate (in deg/s) considered "stable".
    const unsigned long STABLE_TIME = 2000; // Duration (ms) readings must remain stable to consider landed.

    switch (flightState) {
        case PREFLIGHT:
            // If dynamic motion is detected, switch to INFLIGHT.
            if (fabs(a_mag - 1.0f) > ACC_THRESHOLD) {
                flightState = INFLIGHT;
                flightStableStartTime = now;
            }
            break;

        case INFLIGHT:
            // If readings are stable (i.e. near 1g and low gyro) for at least STABLE_TIME,
            // transition to LANDED. Otherwise, reset the timer.
            if (fabs(a_mag - 1.0f) < ACC_THRESHOLD && g_mag < GYRO_THRESHOLD) {
                if ((now - flightStableStartTime) >= STABLE_TIME) {
                    flightState = LANDED;
                }
            } else {
                flightStableStartTime = now;
            }
            break;

        case LANDED:
            // If the aircraft is picked up or thrown again (dynamic motion detected),
            // transition back to INFLIGHT.
            if (fabs(a_mag - 1.0f) > ACC_THRESHOLD) {
                flightState = INFLIGHT;
                flightStableStartTime = now;
            }
            break;

        default:
            // If somehow the state is invalid, default to PREFLIGHT.
            flightState = PREFLIGHT;
            break;
    }
}

/**
 * @brief Retrieves the current flight state.
 * @return The flight state (PREFLIGHT, INFLIGHT, or LANDED).
 */
FlightState ArduFliteIMU::getFlightState() const {
    return flightState;
}