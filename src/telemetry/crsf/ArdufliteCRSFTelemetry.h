/**
 * ArdufliteCRSFTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 * 
 * Sends CRSF telemetry frames in a dedicated FreeRTOS task 
 */
#ifndef ARDUFLITE_CRSF_TELEMETRY_H
#define ARDUFLITE_CRSF_TELEMETRY_H

#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"
#include "src/telemetry/ConfigData.h"

#include <Arduino.h>

/**
 * @class ArdufliteCRSFTelemetry
 * @brief Publishes attitude, link stats, and mode over CRSF telemetry.
 */
class ArdufliteCRSFTelemetry : public ArduFliteTelemetry {
public:
    /**
     * @brief Construct with a UART port for downlink.
     * @param serialPort UART (e.g. Serial2)
     * @param txPin      GPIO pin for CRSF TX line
     * @param freqHz     send frequency (e.g. 25 Hz)
     */
    ArdufliteCRSFTelemetry(HardwareSerial& serialPort,
                           int            txPin,
                           float          freqHz = 25.0f);

    virtual ~ArdufliteCRSFTelemetry();

    /** @see ArduFliteTelemetry */
    void begin() override;

    /** @see ArduFliteTelemetry */
    void publish(const TelemetryData& telem,
                 const ConfigData&    cfg) override;

    /** @brief No-op reset. */
    void reset() override {}

private:
    HardwareSerial&   _serial;
    int               _txPin;
    float             _intervalMs;
    TaskHandle_t      _taskHandle    = nullptr;
    SemaphoreHandle_t _lock          = nullptr;
    TelemetryData     _pendingData;
    ConfigData        _pendingConfig;

    static void telemetryTask(void* pv);
    void run();

    // CRSF framing
    static constexpr uint8_t AddrFC = 0xC8;
    static constexpr uint8_t AddrTX = 0xEE;
    static constexpr uint8_t T_LINK   = 0x14;
    static constexpr uint8_t T_ATT    = 0x1E;
    static constexpr uint8_t T_MODE   = 0x21;

    void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);
    static uint8_t crc8(const uint8_t* data, uint8_t len);

    void sendLinkStats(int8_t rssi, int8_t snr);
    void sendAttitude(const TelemetryData& t);
    void sendFlightMode(const TelemetryData& t);
};

#endif // ARDUFLITE_CRSF_TELEMETRY_H
