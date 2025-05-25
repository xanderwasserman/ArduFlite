// ArduFliteFlashTelemetry.cpp

#include "ArduFliteFlashTelemetry.h"

#define FLUSH_INTERVAL_MS  500
#define MAX_ROW_BUFFER     400

ArduFliteFlashTelemetry::ArduFliteFlashTelemetry(float frequencyHz)
  : _intervalMs(1000.0f / frequencyHz),
    _mutex(nullptr),
    _lastFlushMs(0),
    _isLogging(false)
{}

ArduFliteFlashTelemetry::~ArduFliteFlashTelemetry() {
    if (_mutex) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_isLogging) {
            _logFile.flush();
            _logFile.close();
        }
        xSemaphoreGive(_mutex);
        vSemaphoreDelete(_mutex);
    }
}

void ArduFliteFlashTelemetry::begin() {
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed; formatting...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("LittleFS format failed!");
            return;
        }
    }

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) {
        Serial.println("Failed to create telemetry mutex");
        return;
    }

    FSInfo fs;
    LittleFS.info(fs);
    Serial.printf("TotalBytes: %u\n", fs.totalBytes);
    Serial.printf("UsedBytes:  %u\n", fs.usedBytes);

    // Start the background task
    xTaskCreate(
        telemetryTask,
        "FlashTelTask",
        8192,
        this,
        1,
        nullptr
    );
}

void ArduFliteFlashTelemetry::publish(const TelemetryData& data) {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _pendingData = data;
        xSemaphoreGive(_mutex);
    }
}

int ArduFliteFlashTelemetry::findNextFlightLogIndex() {
    int maxIdx = -1;
    Dir dir = LittleFS.openDir("/");
    while (dir.next()) {
        int idx;
        if (sscanf(dir.fileName(), "/log_%03d.csv", &idx) == 1) {
            maxIdx = max(maxIdx, idx);
        }
    }
    return maxIdx + 1;
}

void ArduFliteFlashTelemetry::startLogging() {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        if (!_isLogging) {
            int idx = findNextFlightLogIndex();
            char fn[32];
            snprintf(fn, sizeof(fn), "/log_%03d.csv", idx);
            _currentFilename = fn;

            _logFile = LittleFS.open(_currentFilename, FILE_WRITE);
            if (_logFile) {
                // write header
                char header[200];
                formatCSVHeader(header, sizeof(header));
                _logFile.write((const uint8_t*)header, strlen(header));
                _logFile.flush();
                _lastFlushMs = millis();
                _isLogging = true;
                Serial.printf("Logging started: %s\n", fn);
            } else {
                Serial.printf("Failed to open %s\n", fn);
            }
        }
        xSemaphoreGive(_mutex);
    }
}

void ArduFliteFlashTelemetry::stopLogging() {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        if (_isLogging) {
            _logFile.flush();
            _logFile.close();
            _isLogging = false;
            Serial.printf("Logging stopped: %s\n", _currentFilename.c_str());
        }
        xSemaphoreGive(_mutex);
    }
}

void ArduFliteFlashTelemetry::listLogs() {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("Available logs:");
        Dir dir = LittleFS.openDir("/");
        while (dir.next()) {
            const char* name = dir.fileName();
            if (strncmp(name, "/log_", 5) == 0) {
                Serial.println(name + 1);  // skip '/'
            }
        }
        xSemaphoreGive(_mutex);
    }
}

void ArduFliteFlashTelemetry::dumpLog(int index) {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        char fn[32];
        snprintf(fn, sizeof(fn), "/log_%03d.csv", index);

        File f = LittleFS.open(fn, FILE_READ);
        if (!f) {
            Serial.printf("Failed to open %s\n", fn);
        } else {
            Serial.printf("\n--- BEGIN %s ---\n", fn);
            while (f.available()) {
                Serial.write(f.read());
            }
            Serial.printf("\n--- END %s ---\n", fn);
            f.close();
        }
        xSemaphoreGive(_mutex);
    }
}

void ArduFliteFlashTelemetry::deleteLog(int index) {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        char fn[32];
        snprintf(fn, sizeof(fn), "/log_%03d.csv", index);
        if (LittleFS.remove(fn)) {
            Serial.printf("Deleted %s\n", fn);
        } else {
            Serial.printf("Failed to delete %s\n", fn);
        }
        xSemaphoreGive(_mutex);
    }
}

void ArduFliteFlashTelemetry::telemetryTask(void* pvParameters) {
    auto* self = static_cast<ArduFliteFlashTelemetry*>(pvParameters);
    char rowBuf[MAX_ROW_BUFFER];

    for (;;) {
        unsigned long ts = millis();
        TelemetryData copy;

        // 1) Copy pendingData under mutex
        if (self->_mutex && xSemaphoreTake(self->_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            copy = self->_pendingData;
            xSemaphoreGive(self->_mutex);
        }

        // 2) Format outside mutex
        size_t len = formatCSVRow(rowBuf, sizeof(rowBuf), ts, copy);

        // 3) Write/flush under mutex if logging
        if (self->_mutex && xSemaphoreTake(self->_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (self->_isLogging && self->_logFile) {
                self->_logFile.write((const uint8_t*)rowBuf, len);
                if (ts - self->_lastFlushMs >= FLUSH_INTERVAL_MS) {
                    self->_logFile.flush();
                    self->_lastFlushMs = ts;
                }
            }
            xSemaphoreGive(self->_mutex);
        }

        // Delay to maintain frequency
        unsigned long elapsed = millis() - ts;
        int delayMs = (int)max(1.0f, self->_intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

void ArduFliteFlashTelemetry::formatCSVHeader(char* buf, size_t bufSize) {
    const char* hdr =
      "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
      "quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,"
      "att_sp_roll,att_sp_pitch,att_sp_yaw,"
      "rate_sp_roll,rate_sp_pitch,rate_sp_yaw,"
      "att_cmd_roll,att_cmd_pitch,att_cmd_yaw,"
      "rate_cmd_roll,rate_cmd_pitch,rate_cmd_yaw,"
      "altitude,flight_state,flight_mode\n";
    strncpy(buf, hdr, bufSize - 1);
    buf[bufSize - 1] = '\0';
}

size_t ArduFliteFlashTelemetry::formatCSVRow(char* buf, size_t bufSize, unsigned long ts, const TelemetryData& d) {
    return snprintf(buf, bufSize,
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%d\n",
        ts,
        d.accel.x, d.accel.y, d.accel.z,
        d.gyro.x, d.gyro.y, d.gyro.z,
        d.quat.w, d.quat.x, d.quat.y, d.quat.z,
        d.orientation.roll, d.orientation.pitch, d.orientation.yaw,
        d.attitudeSetpoint.roll, d.attitudeSetpoint.pitch, d.attitudeSetpoint.yaw,
        d.rateSetpoint.roll, d.rateSetpoint.pitch, d.rateSetpoint.yaw,
        d.attitudeCmd.roll, d.attitudeCmd.pitch, d.attitudeCmd.yaw,
        d.rateCmd.roll, d.rateCmd.pitch, d.rateCmd.yaw,
        d.altitude,
        d.flight_state,
        d.flight_mode
    );
}
