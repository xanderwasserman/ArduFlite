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
 * @brief All the “inbound” CRSF frame types we care about.
 */
static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED   = 0x16;
static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS      = 0x14;
static constexpr uint8_t CRSF_FRAMETYPE_DEVICE_PING          = 0x28;
static constexpr uint8_t CRSF_FRAMETYPE_DEVICE_INFO          = 0x29;
static constexpr uint8_t CRSF_FRAMETYPE_SUBSCRIBE_TELEMETRY  = 0xEE;

/**
 * @brief Payload for Link Statistics (10 bytes).
 * @see   CRSF spec Rev07
 */
typedef struct __attribute__((packed)) {
    uint8_t uplink_RSSI_1;        ///< RSSI of antenna 1 (dBm + 64)
    uint8_t uplink_RSSI_2;        ///< RSSI of antenna 2 (dBm + 64)
    uint8_t uplink_Link_quality;  ///< 0–255 link quality %
    int8_t  uplink_SNR;           ///< uplink SNR (dB)
    uint8_t active_antenna;       ///< 0 or 1
    uint8_t rf_Mode;              ///< RF data rate index
    uint8_t uplink_TX_Power;      ///< TX power index (0–7)
    uint8_t downlink_RSSI;        ///< downlink RSSI (dBm + 64)
    uint8_t downlink_Link_quality;///< downlink link quality %
    int8_t  downlink_SNR;         ///< downlink SNR (dB)
} crsfLinkStatistics_t;

/**
 * @brief Callback signature for updated Link Statistics.
 * @param stats  freshly‐parsed link metrics
 */
using LinkStatsCallback = void(*)(const crsfLinkStatistics_t& stats);

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

    /**
     * @brief Get the latest Link‐Statistics (thread‐safe).
     * @param out  will be filled with the last‐received stats
     * @return     true if we have ever received one
     */
    bool getLinkStats(crsfLinkStatistics_t& out) const;

    /**
     * @brief Check if currently in RC failsafe (link lost).
     * @return true if in failsafe state
     */
    bool isInFailsafe() const;

    /**
     * @brief Register a callback to run when RC failsafe is triggered.
     * @param cb  function to call once upon timeout (and each poll while in failsafe)
     */
    void setFailsafeCallback(void (*cb)());

    /**
     * @brief Register a callback to run when exiting RC failsafe (link restored).
     * @param cb  function to call once when transitioning out of failsafe
     */
    void setFailsafeExitCallback(void (*cb)());

    /**
     * @brief Set the threshold after which the failsafe callback is invoked.
     * @param timeout  The timeout in milliseconds.
     */
    void setFailsafeTimeout(uint32_t timeout);

private:
    HardwareSerial&    _serial;
    int                _rxPin;
    float              _intervalMs;
    TaskHandle_t       _taskHandle  = nullptr;
    SemaphoreHandle_t  _lock        = nullptr;

    // framing buffer
    static constexpr uint8_t  DestFC     = 0xC8;
    static constexpr size_t   MaxFrame   = 64;
    uint8_t  _buf[MaxFrame];
    size_t   _bufLen      = 0;
    size_t   _expectedLen = 0;

    ChannelConfig _chCfg[16];

    // Remember last raw values so we only callback on change
    uint16_t      _lastRaw[16];

    // link‐stats storage
    crsfLinkStatistics_t _latestLinkStats{};
    bool                 _haveLinkStats = false;

    // failsafe
    uint32_t    _failsafeTimeoutMs = 500;   ///< ms without RC until failsafe
    uint32_t    _lastRcMicros     = 0;     ///< micros() of last RC frame
    bool        _inFailsafe       = false; ///< are we currently in failsafe?
    void      (*_failsafeCb)()     = nullptr; ///< user callback on failsafe entry
    void      (*_failsafeExitCb)() = nullptr; ///< user callback on failsafe exit

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
