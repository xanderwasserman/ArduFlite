/**
 * ArdufliteCRSFTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 * 
 * Sends CRSF telemetry frames in a dedicated FreeRTOS task 
 */
#include "src/telemetry/crsf/ArdufliteCRSFTelemetry.h"
#include "src/controller/ArduFliteController.h"

ArdufliteCRSFTelemetry::ArdufliteCRSFTelemetry(HardwareSerial& ser, int txPin, float freqHz)
  : _serial(ser)
  , _txPin(txPin)
  , _intervalMs(1000.0f/freqHz)
{
    _lock = xSemaphoreCreateMutex();
}

ArdufliteCRSFTelemetry::~ArdufliteCRSFTelemetry() {
    if (_taskHandle) vTaskDelete(_taskHandle);
    if (_lock)       vSemaphoreDelete(_lock);
}

void ArdufliteCRSFTelemetry::begin() {
    // 420 000 baud, 8N1, TX only
    _serial.begin(420000, SERIAL_8N1, -1, _txPin);
    if (_taskHandle) return;

    BaseType_t res = xTaskCreate(
        telemetryTask,
        "CrsfTelem",
        4096,
        this,
        tskIDLE_PRIORITY+1,
        &_taskHandle
    );
    if (res != pdPASS) {
        // handle error…
    }
}

void ArdufliteCRSFTelemetry::publish(const TelemetryData& telem, const ConfigData& cfg)
{
    if (_lock && xSemaphoreTake(_lock, 10) == pdTRUE) {
        _pendingData   = telem;
        _pendingConfig = cfg;
        xSemaphoreGive(_lock);
    }
}

void ArdufliteCRSFTelemetry::telemetryTask(void* pv) {
    auto* self = static_cast<ArdufliteCRSFTelemetry*>(pv);
    self->run();
}

void ArdufliteCRSFTelemetry::run() {
    while (true) {
        TelemetryData td;
        ConfigData    cd;
        if (xSemaphoreTake(_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
            td = _pendingData;
            cd = _pendingConfig;
            xSemaphoreGive(_lock);
        }

        // send three frames
        sendLinkStats( (int8_t)td.flight_state,
                       (int8_t)td.flight_mode );
        sendAttitude(td);
        sendFlightMode(td);

        vTaskDelay(pdMS_TO_TICKS(_intervalMs));
    }
}

uint8_t ArdufliteCRSFTelemetry::crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

void ArdufliteCRSFTelemetry::sendFrame(uint8_t type,
                                       const uint8_t* payload,
                                       uint8_t len)
{
    uint8_t flen = len + 2;  // type + payload + CRC
    _serial.write(AddrTX);
    _serial.write(flen);
    _serial.write(type);
    _serial.write(payload, len);
    uint8_t c = crc8(&type, len+1);
    _serial.write(c);
}

void ArdufliteCRSFTelemetry::sendLinkStats(int8_t rssi, int8_t snr) {
    uint8_t buf[2] = { (uint8_t)rssi, (uint8_t)snr };
    sendFrame(T_LINK, buf, 2);
}

void ArdufliteCRSFTelemetry::sendAttitude(const TelemetryData& t) {
    int16_t p = int16_t(t.orientation.pitch * 10000.0f);
    int16_t r = int16_t(t.orientation.roll  * 10000.0f);
    int16_t y = int16_t(t.orientation.yaw   * 10000.0f);
    uint8_t buf[6];
    memcpy(buf,   &p, 2);
    memcpy(buf+2, &r, 2);
    memcpy(buf+4, &y, 2);
    sendFrame(T_ATT, buf, 6);
}

void ArdufliteCRSFTelemetry::sendFlightMode(const TelemetryData& t) {
    const char* modes[] = {
        "ASSIST","STABALIZED","UNKNOWN",nullptr
    };
    uint8_t idx = (t.flight_mode < FLIGHT_MODE_LENGTH) ? t.flight_mode : UNKNOWN_MODE;
    const char* txt = modes[idx] ? modes[idx] : "UNKNOWN";
    sendFrame(T_MODE, (const uint8_t*)txt, strlen(txt)+1);
}
