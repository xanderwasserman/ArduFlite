/**
 * @file      ArdufliteCRSFTelemetry.h
 * @brief     CRSF telemetry interface for ArduFlite:
 *            publishes attitude, link‐stats, battery, GPS, vario, and flight‐mode
 *            in a dedicated FreeRTOS task over a HardwareSerial port.
 *
 * @author    Alexander Wasserman
 * @version   1.1
 * @date      07 June 2025
 *
 * @copyright MIT License
 */

#ifndef ARDUFLITE_CRSF_TELEMETRY_H
#define ARDUFLITE_CRSF_TELEMETRY_H

#include <Arduino.h>
#include <atomic>
#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"

/**
 * @class   ArdufliteCRSFTelemetry
 * @brief   Runs a FreeRTOS task that serializes TelemetryData into
 *          CRSF frames and sends them via a UART (e.g. to ExpressLRS/CRSF RX).
 *
 * @details Supports Link‐Stats, Attitude, Flight Mode, Battery, GPS, and Vario frames.
 */
class ArdufliteCRSFTelemetry : public ArduFliteTelemetry {
public:
    /**
     * @brief Construct a new CRSF telemetry publisher.
     * @param[in] serialPort  UART port used for CRSF (shared with receiver).
     * @param[in] freqHz      Overall send frequency in Hz (default 1 Hz).
     * @note  UART must be initialized by receiver's begin() before this class is started.
     */
    ArdufliteCRSFTelemetry(HardwareSerial& serialPort,
                           float          freqHz = 1.0f);

    /**
     * @brief Destroy the CRSF telemetry object.
     *        Stops the task and deletes the mutex.
     */
    virtual ~ArdufliteCRSFTelemetry();

    /**
     * @brief Start telemetry task (UART already configured by receiver).
     */
    void begin() override;

    /**
     * @brief Supply a fresh snapshot of telemetry data.
     * @param[in] telem  Latest sensor & state data.
     */
    void publish(const TelemetryData& telem) override;

    /**
     * @brief Reset telemetry state (no-op).
     */
    void reset() override {}

    /**
     * @brief Pause the telemetry task (unsubscribes from WDT).
     *        Use during calibration to prevent WDT timeout.
     */
    void pauseTask();

    /**
     * @brief Resume the telemetry task (re-subscribes to WDT).
     */
    void resumeTask();

private:
    HardwareSerial&   _serial;        /**< UART port for CRSF (shared with receiver). */
    float             _intervalMs;    /**< Delay between batches, in milliseconds. */
    TaskHandle_t      _taskHandle{};  /**< FreeRTOS task handle. */
    SemaphoreHandle_t _lock{};        /**< Protects _pendingData. */
    TelemetryData     _pendingData;   /**< Last-published TelemetryData. */
    std::atomic<bool> _paused{false}; /**< When true, task skips work and WDT is unsubscribed. */

    /**
     * @brief FreeRTOS task entry point.
     * @param pv  Pointer to this instance.
     */
    static void telemetryTask(void* pv);

    /**
     * @brief Main loop: pulls the latest snapshot, sends all frames, delays.
     */
    void run();

    // CRSF addressing & frame‐type constants:
    static constexpr uint8_t AddrFC    = 0xC8;  /**< Flight controller address (FC→TX direction). */
    static constexpr uint8_t AddrRX    = 0xEA;  /**< Receiver address (FC→RX telemetry frames). */
    static constexpr uint8_t T_LINK    = 0x14;  /**< Link statistics. */
    static constexpr uint8_t T_ATT     = 0x1E;  /**< Attitude (6 bytes). */
    static constexpr uint8_t T_MODE    = 0x21;  /**< Flight mode (string). */
    static constexpr uint8_t T_BATTERY = 0x08;  /**< Battery sensor. */
    static constexpr uint8_t T_GPS     = 0x02;  /**< GPS data. */
    static constexpr uint8_t T_VARIO   = 0x07;  /**< Vario (climb rate). */
    static constexpr uint8_t T_BARO_ALT = 0x09; /**< Barometric altitude. */

    /**
     * @brief Send a generic CRSF frame.
     * @param[in] type     CRSF frame-type.
     * @param[in] payload  Pointer to payload bytes.
     * @param[in] len      Payload length in bytes.
     */
    void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);

    /**
     * @brief Compute the CRSF CRC‐8 (poly 0xD5).
     * @param[in] data  Pointer to (type + payload).
     * @param[in] len   Number of bytes to checksum.
     * @returns   8-bit CRC.
     */
    static uint8_t crc8(const uint8_t* data, uint8_t len);

    /**
     * @brief Send link statistics (RSSI, SNR, quality, etc.).
     * @param[in] t  TelemetryData containing link fields.
     */
    void sendLinkStats(const TelemetryData& t);

    /**
     * @brief Send attitude (pitch, roll, yaw).
     * @param[in] t  TelemetryData containing orientation.
     */
    void sendAttitude(const TelemetryData& t);

    /**
     * @brief Send current flight mode as a null-terminated string.
     * @param[in] t  TelemetryData containing flight_mode index.
     */
    void sendFlightMode(const TelemetryData& t);

    /**
     * @brief Send battery data (voltage, current, consumed, remaining%).
     * @param[in] t  TelemetryData containing battery fields.
     */
    void sendBattery(const TelemetryData& t);

    /**
     * @brief Send GPS fix, speed, heading, altitude, satellite count.
     * @param[in] t  TelemetryData containing GPS fields.
     */
    void sendGps(const TelemetryData& t);

    /**
     * @brief Send vario (vertical speed) in m/s.
     * @param[in] t  TelemetryData containing climb_rate.
     */
    void sendVario(const TelemetryData& t);

    /**
     * @brief Send barometric altitude.
     * @param[in] t  TelemetryData containing altitude.
     */
    void sendBaroAlt(const TelemetryData& t);
};

#endif // ARDUFLITE_CRSF_TELEMETRY_H
