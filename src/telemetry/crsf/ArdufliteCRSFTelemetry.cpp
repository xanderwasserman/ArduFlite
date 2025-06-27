/**
 * @file      ArdufliteCRSFTelemetry.cpp
 * @brief     Implementation of CRSF telemetry over a dedicated FreeRTOS task.
 * @author    Alexander Wasserman
 * @version   1.1
 * @date      07 June 2025
 *
 * @copyright MIT License
 *
 * This module serializes telemetry data into CRSF frames and sends them
 * at a fixed rate over a HardwareSerial port (e.g. to an ELRS or Crossfire
 * receiver). Uses a FreeRTOS task and mutex to decouple from the FC's
 * main control loop.
 */

#include "src/telemetry/crsf/ArdufliteCRSFTelemetry.h"
#include "src/controller/ArduFliteController.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

#include <math.h> 
#include <cstring> 

/// @brief Constructor.
/// @param[in] ser     Reference to a HardwareSerial port for CRSF downlink.
/// @param[in] txPin   GPIO pin to use as UART TX line.
/// @param[in] freqHz  Frame-send frequency, in Hz (default 25Hz).
ArdufliteCRSFTelemetry::ArdufliteCRSFTelemetry(HardwareSerial& ser,
                                               int            txPin,
                                               float          freqHz)
  : _serial(ser)
  , _txPin(txPin)
  , _intervalMs(1000.0f / freqHz)
{
    _lock = xSemaphoreCreateMutex();
    if (!_lock) 
    {
        LOG_ERR("CRSF Telemetry: failed to create mutex");
    } 
    else 
    {
        LOG_DBG("CRSF Telemetry: mutex created");
    }
}

/// @brief Destructor: stops the telemetry task and deletes the mutex.
ArdufliteCRSFTelemetry::~ArdufliteCRSFTelemetry()
{
    if (_taskHandle) 
    {
        vTaskDelete(_taskHandle);
        LOG_INF("CRSF Telemetry: task deleted");
    }
    if (_lock) 
    {
        vSemaphoreDelete(_lock);
        LOG_INF("CRSF Telemetry: mutex deleted");
    }
}

/// @brief Initializes the UART and starts the telemetry FreeRTOS task.
/// @note  Baud = 420000, 8N1. TX only.
/// @see   xTaskCreate()
void ArdufliteCRSFTelemetry::begin()
{
    LOG_INF("CRSF Telemetry: starting on TX pin %d @ 420000 baud", _txPin);
    _serial.begin(420000, SERIAL_8N1, /*rxPin=*/-1, _txPin);

    if (_taskHandle) 
    {
        LOG_WARN("CRSF Telemetry: begin() called but task already running");
        return;  ///< already running
    }

    BaseType_t res = xTaskCreate(
        telemetryTask,
        "CrsfTelem",
        /*stack depth*/ 4096,
        this,
        tskIDLE_PRIORITY + 1,
        &_taskHandle
    );
    if (res != pdPASS) 
    {
        LOG_ERR("CRSF Telemetry: failed to start task (pdPASS=%d)", pdPASS);
    } 
    else 
    {
        LOG_INF("CRSF Telemetry: task started (interval %.1f ms)", _intervalMs);
    }

    LOG_INF("Started CRSF Telemetry task.");
}

/// @brief Publishes new telemetry & config snapshots.
/// @param[in] telem  Fresh sensor & state data.
/// @param[in] cfg    Configuration data (not used in this version).
/// @note  Uses a mutex to safely swap in the pending buffers.
void ArdufliteCRSFTelemetry::publish(const TelemetryData& telem,
                                     const ConfigData&    cfg)
{
    if (!_lock) return;

    {
        SemaphoreLock lock(_lock);
        _pendingData   = telem;
        _pendingConfig = cfg;
    }
}

/// @brief FreeRTOS task entrypoint.
/// @param[in] pv  Pointer to the ArdufliteCRSFTelemetry instance.
void ArdufliteCRSFTelemetry::telemetryTask(void* pv)
{
    auto* self = static_cast<ArdufliteCRSFTelemetry*>(pv);
    LOG_INF("CRSF Telemetry: task entry");
    self->run();
}

/// @brief Main loop: pulls the latest buffer, sends all frames, then delays.
/// @details
///   - Attitude, link stats, battery, GPS, vario, flight mode.
///   - Runs at _intervalMs configured in constructor.
void ArdufliteCRSFTelemetry::run()
{
    LOG_DBG("CRSF Telemetry: run() loop begin");

    // Keep track of when we last sent the slow (1 Hz) frames:
    uint32_t lastSlowTime = millis();

    while (true) 
    {
        uint32_t now = millis();

        // 1) grab a snapshot
        TelemetryData td;
        ConfigData    cd;
        if (_lock) 
        {
            SemaphoreLock lock(_lock);
            td = _pendingData;
            cd = _pendingConfig;
        }

        // 2) send “fast” (vital) frames every loop
        sendLinkStats(td);
        sendVario    (td);
        sendAttitude (td);

        // 3) send “slow” frames at ~1 Hz
        if (now - lastSlowTime >= 1000) 
        {
            LOG_DBG("CRSF Telemetry: sending 1 Hz frames");
            sendBattery   (td);
            sendGps       (td);
            sendFlightMode(td);
            lastSlowTime = now;
        }

        // 4) wait until next burst
        vTaskDelay(pdMS_TO_TICKS(_intervalMs));
    }
}


/// @brief Computes the standard CRC-8 for CRSF (poly 0xD5).
/// @param[in] data  Byte array to checksum (type byte + payload).
/// @param[in] len   Number of bytes in @p data.
/// @returns 8-bit CRC.
uint8_t ArdufliteCRSFTelemetry::crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i) {
            crc = (crc & 0x80)
                  ? (uint8_t)((crc << 1) ^ 0xD5u)
                  : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/// @brief Sends a generic CRSF frame.
/// @param[in] type     CRSF frame type byte (e.g. T_ATT).
/// @param[in] payload  Pointer to the payload bytes.
/// @param[in] len      Number of payload bytes.
/// @note  Prepends AddrTX and length, appends CRC8(type|payload).
void ArdufliteCRSFTelemetry::sendFrame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    // Length = type (1) + payload (len) + CRC (1)
    uint8_t flen = len + 2;
    LOG_DBG("CRSF Telemetry: frame 0x%02X, payload %u bytes, flen %u", type, len, flen);

    _serial.write(AddrTX);
    _serial.write(flen);
    _serial.write(type);
    _serial.write(payload, len);

    // CRC covers type + payload
    uint8_t c = crc8(&type, len + 1);
    _serial.write(c);
}

/// @brief Sends the 6-byte attitude frame (pitch, roll, yaw).
/// @param[in] t  Latest telemetry (angles converted to radians).
void ArdufliteCRSFTelemetry::sendAttitude(const TelemetryData& t)
{
    // Convert from degrees → radians, then scale
    constexpr float DEG2RAD = M_PI / 180.0f;
    constexpr float SCALE     = 10000.0f;
    
    int16_t p = int16_t((t.orientation.pitch * DEG2RAD) * SCALE);
    int16_t r = int16_t((t.orientation.roll  * DEG2RAD) * SCALE);
    int16_t y = int16_t((t.orientation.yaw   * DEG2RAD) * SCALE);

    uint8_t buf[6];
    buf[0] = uint8_t(p & 0xFF);
    buf[1] = uint8_t((p >> 8) & 0xFF);
    buf[2] = uint8_t(r & 0xFF);
    buf[3] = uint8_t((r >> 8) & 0xFF);
    buf[4] = uint8_t(y & 0xFF);
    buf[5] = uint8_t((y >> 8) & 0xFF);

    LOG_DBG("CRSF Telemetry: Attitude P=%d R=%d Y=%d", p, r, y);
    sendFrame(T_ATT, buf, sizeof(buf));
}


/// @brief Sends the null-terminated flight mode string.
/// @param[in] t  Latest telemetry (mode index).
void ArdufliteCRSFTelemetry::sendFlightMode(const TelemetryData& t)
{
    static const char* modes[] = {
        "ATTI", "RATE", "UNKN", nullptr
    };
    uint8_t idx = (t.flight_mode < FLIGHT_MODE_LENGTH) ? uint8_t(t.flight_mode) : UNKNOWN_MODE;
    const char* txt = (modes[idx]) ? modes[idx] : "UNKN";

    LOG_DBG("CRSF Telemetry: FlightMode \"%s\"", txt);
    sendFrame(T_MODE, reinterpret_cast<const uint8_t*>(txt), uint8_t(strlen(txt) + 1));
}

/// @brief Sends battery status (voltage/current/capacity/%).
/// @param[in] t  Latest telemetry (battery fields populated).
void ArdufliteCRSFTelemetry::sendBattery(const TelemetryData& t)
{
    uint8_t buf[8];
    uint16_t v   = uint16_t(t.battery_voltage * 1000.0f);
    uint16_t i   = uint16_t(t.battery_current * 1000.0f);
    uint32_t cap = t.battery_consumed;

    buf[0] = uint8_t(v & 0xFF);       // LSB
    buf[1] = uint8_t(v >> 8);         // MSB
    // same for current
    buf[2] = uint8_t(i & 0xFF);
    buf[3] = uint8_t(i >> 8);
    // capacity is 24-bit LSB first:
    buf[4] = uint8_t( cap        & 0xFF);
    buf[5] = uint8_t((cap >>  8) & 0xFF);
    buf[6] = uint8_t((cap >> 16) & 0xFF);
    buf[7] =    t.battery_remaining;

    LOG_DBG("CRSF Telemetry: Battery V=%u mV I=%u mA C=%u%%", v, i, t.battery_remaining);
    sendFrame(T_BATTERY, buf, sizeof(buf));
}

/// @brief Sends GPS fix, speed, heading, altitude, sats.
/// @param[in] t  Latest telemetry (GPS fields populated).
void ArdufliteCRSFTelemetry::sendGps(const TelemetryData& t)
{
    uint8_t buf[15];
    int32_t lat = int32_t(t.gps_lat * 1e7);
    int32_t lon = int32_t(t.gps_lon * 1e7);
    uint16_t gs = uint16_t(t.gps_groundspeed * 100.0f);
    uint16_t hd = uint16_t(t.gps_heading      * 100.0f);
    uint16_t al = uint16_t(t.gps_alt + 1000.0f);

    memcpy(buf + 0, &lat, 4);
    memcpy(buf + 4, &lon, 4);

    buf[ 8] = uint8_t(gs & 0xFF);
    buf[ 9] = uint8_t(gs >> 8);

    buf[10] = uint8_t(hd & 0xFF);    
    buf[11] = uint8_t(hd >> 8);

    buf[12] = uint8_t(al & 0xFF);   
    buf[13] = uint8_t(al >> 8); 

    buf[14] = t.gps_sats;

    LOG_DBG("CRSF Telemetry: GPS lat=%ld lon=%ld gs=%u hd=%u al=%u sats=%u", lat, lon, gs, hd, al, t.gps_sats);
    sendFrame(T_GPS, buf, sizeof(buf));
}

/// @brief Sends vertical speed (vario) in cm/s.
/// @param[in] t  Latest telemetry (climb_rate in m/s).
void ArdufliteCRSFTelemetry::sendVario(const TelemetryData& t)
{
    int16_t vs = int16_t(t.climb_rate * 100.0f);
    uint8_t buf[2] = { uint8_t(vs & 0xFF), uint8_t(vs >> 8) };

    LOG_DBG("CRSF Telemetry: Vario %d cm/s", vs);
    sendFrame(T_VARIO, buf, sizeof(buf));
}

/// @brief Sends full link statistics (10-byte struct).
/// @param[in] t  Latest telemetry (link fields populated).
void ArdufliteCRSFTelemetry::sendLinkStats(const TelemetryData& t)
{
    crsfLinkStatistics_t s = {
        .uplink_RSSI_1         = t.link_rssi1,
        .uplink_RSSI_2         = t.link_rssi2,
        .uplink_Link_quality   = t.link_quality,
        .uplink_SNR            = int8_t(t.link_snr),
        .active_antenna        = t.link_antenna,
        .rf_Mode               = t.link_rf_mode,
        .uplink_TX_Power       = t.link_tx_power,
        .downlink_RSSI         = t.dl_rssi,
        .downlink_Link_quality = t.dl_quality,
        .downlink_SNR          = int8_t(t.dl_snr)
    };

    LOG_DBG("CRSF Telemetry: Link RSSI1=%d RSSI2=%d Q=%u SNR=%d ANT=%u", s.uplink_RSSI_1, s.uplink_RSSI_2, s.uplink_Link_quality, s.uplink_SNR, s.active_antenna);
    sendFrame(T_LINK, reinterpret_cast<const uint8_t*>(&s), sizeof(s));
}
