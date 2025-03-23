#include "src/orientation/ArduFliteIMU.h"

//////////////////////////////////////
// Constructor
//////////////////////////////////////
ArduFliteIMU::ArduFliteIMU() {
    // Initialize our offsets to zero by default
    offsets = {0, 0, 0, 0, 0, 0};
}

//////////////////////////////////////
// Initialization
//////////////////////////////////////
bool ArduFliteIMU::begin() {
    // For ESP32, ensure we can read/write EEPROM
    EEPROM.begin(EEPROM_SIZE);
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C, as in example

    // Initialize IMU with calibration data (which is 0 if not yet calibrated)
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("FastIMU init error: ");
        Serial.println(err);
        return false;
    }

    //Set the sensor orientation
    err = IMU.setGyroRange(500);  
    err = IMU.setAccelRange(4);

    if (err == -1) {
      Serial.print("Error setting IMU sensor ranges: ");
      Serial.println(err);
      return false;
    }

    // Load calibration offsets from EEPROM (if available)
    if (!applyCalibrations()) {
        return false;
    }

    // Setup & Warm up the Madgwick filter before normal operation
    initMadgwickFilter();

    Serial.println("FastIMU (MPU-6500) initialized!");
    return true;
}

//////////////////////////////////////
// selfCalibrate()
//      Our own approach to gather raw 
//      data for ~2s, compute offsets, 
//      store them, and save to EEPROM
//////////////////////////////////////
bool ArduFliteIMU::selfCalibrate() {
    Serial.println("=== Self Calibration Start ===");
    Serial.println("Please keep IMU still & level in final orientation...");

    const unsigned long CALIB_MS = 10000;
    unsigned long start = millis();
    unsigned int samples = 0;

    double sumAx = 0, sumAy = 0, sumAz = 0;
    double sumGx = 0, sumGy = 0, sumGz = 0;

    while (millis() - start < CALIB_MS) {
        // Read raw sensor data
        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        double ax = accelData.accelX;
        double ay = accelData.accelY;
        double az = accelData.accelZ;
        double gx = gyroData.gyroX;
        double gy = gyroData.gyroY;
        double gz = gyroData.gyroZ;

        // Accumulate the readings
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        samples++;

        delay(5); // small delay between samples
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

    // Decide desired final orientation:
    // We want final ax ~ 0, ay ~ 0, az ~ 1 (for "Z down" at rest),
    // and gyro readings all near 0.
    ArduFliteIMUOffsets newOfs;
    // For X and Y: offset = avg (since desired is 0)
    newOfs.accelX = float(avgAx - 0.0);
    newOfs.accelY = float(avgAy - 0.0);
    // For Z: offset = avgAz - (desired), with desired = 1
    newOfs.accelZ = float(avgAz - 1.0);

    // For gyro, simply set offset = average (desired is 0)
    newOfs.gyroX  = float(avgGx);
    newOfs.gyroY  = float(avgGy);
    newOfs.gyroZ  = float(avgGz);

    Serial.println("Computed new offsets:");
    Serial.print("Accel Offsets: ");
    Serial.print(newOfs.accelX, 3); 
    Serial.print(", ");
    Serial.print(newOfs.accelY, 3); 
    Serial.print(", ");
    Serial.println(newOfs.accelZ, 3);

    Serial.print("Gyro Offsets: ");
    Serial.print(newOfs.gyroX, 3); 
    Serial.print(", ");
    Serial.print(newOfs.gyroY, 3); 
    Serial.print(", ");
    Serial.println(newOfs.gyroZ, 3);

    // Store in our class and save to EEPROM
    setOffsets(newOfs);
    saveOffsetsToEEPROM(newOfs);

    Serial.println("=== Self Calibration Done ===");
    return true;
}

//////////////////////////////////////
// update()
//////////////////////////////////////
void ArduFliteIMU::update(float dt) 
{
  IMU.update();

  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  // Subtract offsets
  accelX = accelData.accelX - offsets.accelX;
  accelY = accelData.accelY - offsets.accelY;
  accelZ = accelData.accelZ - offsets.accelZ;

  gyroX = gyroData.gyroX - offsets.gyroX;
  gyroY = gyroData.gyroY - offsets.gyroY;
  gyroZ = gyroData.gyroZ - offsets.gyroZ;

  // Apply orientation transforms
  applyOrientation();

  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);

  filter.getQuaternion(&qw, &qx, &qy, &qz);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
}

//////////////////////////////////////
// 'Warm-up' the Madgwick filter so that
//  it produces stable results when we
// start using it.
//////////////////////////////////////
void ArduFliteIMU::initMadgwickFilter() 
{   
    // Begin the filter with an initial sample frequency (2 Hz in this example)
    filter.begin(FILTER_UPDATE_RATE_HZ); 

    // Run a number of update iterations to let the filter settle..
    unsigned long lastMicros = 0;
    for (int i = 0; i < 2000; i++) {
      // Calculate delta time
      unsigned long currentMicros = micros();
      float dt = (currentMicros - lastMicros) / 1000000.0f;
      lastMicros = currentMicros;

      IMU.update();
      IMU.getAccel(&accelData);
      IMU.getGyro(&gyroData);

      // Subtract offsets
      accelX = accelData.accelX - offsets.accelX;
      accelY = accelData.accelY - offsets.accelY;
      accelZ = accelData.accelZ - offsets.accelZ;

      gyroX = gyroData.gyroX - offsets.gyroX;
      gyroY = gyroData.gyroY - offsets.gyroY;
      gyroZ = gyroData.gyroZ - offsets.gyroZ;   

      applyOrientation();

      filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
  }
  Serial.println("Madgwick filter warm-up complete.");
}

//////////////////////////////////////
// applyOrientation()
//////////////////////////////////////
void ArduFliteIMU::applyOrientation() {
#if (IMU_ORIENTATION == ORIENTATION_NORMAL)
    //pass

#elif (IMU_ORIENTATION == ORIENTATION_SENSOR_FLIPPED_YZ)
    // accelX = -accelX;
    gyroX = -gyroX;
    
    accelY = -accelY;
    // gyroY = -gyroY;

    // accelZ = -accelZ;
    gyroZ = -gyroZ;
#else
    #error "Unknown IMU_ORIENTATION selected!"
#endif
}

//////////////////////////////////////
// setOffsets(), getOffsets()
//////////////////////////////////////
void ArduFliteIMU::setOffsets(const ArduFliteIMUOffsets &ofs) {
    offsets = ofs;
}

void ArduFliteIMU::getOffsets(ArduFliteIMUOffsets &ofs) const {
    ofs = offsets;
}

//////////////////////////////////////
// 9) load/save to EEPROM
//////////////////////////////////////
bool ArduFliteIMU::loadOffsetsFromEEPROM(ArduFliteIMUOffsets &dest) {
    StoredCalibData tmp;
    EEPROM.get(CALIB_DATA_ADDR, tmp);
    if (tmp.magic != CALIB_MAGIC) {
        return false; // no valid data
    }
    dest = tmp.offsets;
    return true;
}

void ArduFliteIMU::saveOffsetsToEEPROM(const ArduFliteIMUOffsets &ofs) {
    StoredCalibData tmp;
    tmp.magic = CALIB_MAGIC;
    tmp.offsets = ofs;
    EEPROM.put(CALIB_DATA_ADDR, tmp);
    EEPROM.commit();  // ESP32 style
    Serial.println("Calibration data saved to EEPROM.");
}

//////////////////////////////////////
// applyCalibrations()
// Loads calibration data from EEPROM and applies them.
//////////////////////////////////////
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
