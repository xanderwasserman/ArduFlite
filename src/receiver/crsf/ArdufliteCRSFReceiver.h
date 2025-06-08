/**
 * ArdufliteCRSFReceiver.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 * 
 * Receives and decodes CRSF RC channels in a dedicated FreeRTOS task.
 */

#ifndef ARDUFLITE_CRSF_RECEIVER_H
#define ARDUFLITE_CRSF_RECEIVER_H

#include <Arduino.h>

/**
 * @brief Types of channel mappings.
 */
enum class ChannelType : uint8_t {
    Raw,        ///< 0…2047
    DualThrow,  ///< -1…+1
    SingleThrow,///< 0…+1
    Boolean,    ///< 0 or 1
    TriState,   ///< -1, 0, +1
    Custom      ///< user-provided converter()
};

/**
 * @brief Signature for a custom raw→float converter.
 * @param rawValue 11-bit raw channel value (0…2047)
 * @return      mapped float
 */
using ChannelConverter = float(*)(uint16_t rawValue);

/**
 * @brief Signature for per-channel change callbacks.
 * @param channelIdx 0…15
 * @param value      mapped value
 */
using ChannelCallback  = void (*)(uint8_t channelIdx, float value);

/**
 * @brief Configuration for one CRSF channel.
 */
struct ChannelConfig {
    ChannelType     type           = ChannelType::Raw;
    float           thrLow         = 0.33f;   ///< for TriState
    float           thrHigh        = 0.66f;   ///< for TriState
    ChannelConverter converter     = nullptr; ///< for Custom
    ChannelCallback  callback      = nullptr; ///< invoked on new data
};

/**
 * @class ArdufliteCRSFReceiver
 * @brief Parses CRSF RC frames off a UART in its own FreeRTOS task.
 */
class ArdufliteCRSFReceiver {
public:
    /**
     * @brief Construct with a hardware serial instance.
     * @param serialPort UART port (e.g. Serial1)
     * @param rxPin      GPIO pin for CRSF RX line
     * @param freqHz     polling frequency (defaults to 500 Hz)
     */
    ArdufliteCRSFReceiver(HardwareSerial& serialPort,
                          int            rxPin,
                          float          freqHz = 500.0f);

    /** @brief Release resources. */
    ~ArdufliteCRSFReceiver();

    /**
     * @brief Initialize UART and launch the parser task.
     *        Must be called once from setup().
     */
    void begin();

    /**
     * @brief Set mapping and callback for one channel.
     * @param idx 0…15
     * @param cfg configuration struct
     */
    void configureChannel(uint8_t idx, const ChannelConfig& cfg);

private:
    HardwareSerial&    _serial;
    int                _rxPin;
    float              _intervalMs;
    TaskHandle_t       _taskHandle = nullptr;
    SemaphoreHandle_t  _lock;

    // framing buffer
    static constexpr uint8_t  DestFC     = 0xC8;
    static constexpr uint8_t  TypeRC     = 0x16;
    static constexpr size_t   MaxFrame   = 64;
    uint8_t  _buf[MaxFrame];
    size_t   _bufLen      = 0;
    size_t   _expectedLen = 0;

    ChannelConfig _chCfg[16];

    // FreeRTOS entrypoint
    static void taskLoop(void* pv);

    // instance main loop
    void run();

    // byte-at-a-time parser
    void parseByte(uint8_t b);

    // after full frame + CRC OK
    void dispatchFrame(const uint8_t* frame, size_t len);

    // RC frame handler
    void handleRC(const uint8_t* payload, size_t);

    // decode 16×11bit values
    void decodeChannels(const uint8_t* p, uint16_t out[16]);

    // mapping raw→float
    float applyMapping(const ChannelConfig& cfg, uint16_t raw);

    // CRC8 (poly 0xD5)
    static uint8_t crc8(const uint8_t* data, uint8_t len);
};

#endif // ARDUFLITE_CRSF_RECEIVER_H
