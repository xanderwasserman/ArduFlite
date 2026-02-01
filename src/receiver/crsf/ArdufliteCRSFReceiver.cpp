/**
 * ArdufliteCRSFReceiver.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 * 
 * Receives and decodes CRSF RC channels in a dedicated FreeRTOS task.
 */

#include "src/receiver/crsf/ArdufliteCRSFReceiver.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

#include <cstring>

ArdufliteCRSFReceiver::ArdufliteCRSFReceiver(HardwareSerial& ser, int rxPin, float freqHz)
  : _serial(ser)
  , _rxPin(rxPin)
  , _intervalMs(1000.0f/freqHz)
{
    _lock = xSemaphoreCreateMutex();
    // initialize lastRaw to invalid so first reading always fires
    for (auto &v : _lastRaw) v = 0xFFFF;  // force first‐time callbacks
}

ArdufliteCRSFReceiver::~ArdufliteCRSFReceiver() 
{
    if (_taskHandle) vTaskDelete(_taskHandle);
    if (_lock)       vSemaphoreDelete(_lock);
}

void ArdufliteCRSFReceiver::begin() 
{
    // 420 000 baud, 8N1, RX only
    _serial.begin(420000, SERIAL_8N1, _rxPin, -1);
    if (_taskHandle) return;

    BaseType_t res = xTaskCreate(
        taskLoop,
        "CRSFRecv",
        4096,
        this,
        tskIDLE_PRIORITY+2,  // Priority 2: above telemetry/CLI, below control loops
        &_taskHandle
    );
    if (res != pdPASS) 
    {
        LOG_ERR("Failed to create CRSFRecv Task!");
    }
}

void ArdufliteCRSFReceiver::configureChannel(uint8_t idx, const ChannelConfig& cfg) 
{
    LOG_DBG("Configured channel: %u", idx);
    if (idx >= 16) return;

    {
        SemaphoreLock lock(_lock);
        _chCfg[idx] = cfg;
    }
}

void ArdufliteCRSFReceiver::setFailsafeCallback(void (*cb)()) 
{
    SemaphoreLock lock(_lock);
    _failsafeCb = cb;
}

void ArdufliteCRSFReceiver::setFailsafeTimeout(uint32_t timeout)
{
    SemaphoreLock lock(_lock);
    _failsafeTimeoutMs = timeout;
}

bool ArdufliteCRSFReceiver::getLinkStats(crsfLinkStatistics_t& out) const
{
    bool ok = false;

    {
        SemaphoreLock lock(_lock);
        if (_haveLinkStats) 
        {
            out = _latestLinkStats;
            ok = true;
        }
    }

    return ok;
}

void ArdufliteCRSFReceiver::taskLoop(void* pv) 
{
    static_cast<ArdufliteCRSFReceiver*>(pv)->run();
}

void ArdufliteCRSFReceiver::run() 
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(_intervalMs);

    while (true) 
    {
        while (_serial.available()) 
        {
            parseByte((uint8_t)_serial.read());
        }

        // check for RC failsafe
        {
            uint32_t now = micros();
            SemaphoreLock lock(_lock);
            if (_lastRcMicros != 0 && (now - _lastRcMicros) > _failsafeTimeoutMs * 1000u)
            {
                if (!_inFailsafe) 
                {
                    LOG_WARN("Entering RC failsafe");
                    _inFailsafe = true;
                    _failsafeCb();
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

uint8_t ArdufliteCRSFReceiver::crc8(const uint8_t* data, uint8_t len) 
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

void ArdufliteCRSFReceiver::parseByte(uint8_t b) 
{
    LOG_DBG("Received: %u", b);
    if (_bufLen == 0) 
    {
        if (b != DestFC) return;
        _buf[0] = b; 
        _bufLen = 1;
        return;
    }
    _buf[_bufLen++] = b;
    if (_bufLen == 2) 
    {
        _expectedLen = b + 2;
        if (_expectedLen > MaxFrame) _bufLen = 0;
        return;
    }

    if (_bufLen < _expectedLen) return;

    // full frame
    uint8_t type = _buf[2];
    uint8_t crcR = _buf[_expectedLen-1];

    if (crc8(_buf+2, _expectedLen-3) == crcR) 
    {
        dispatchFrame(_buf, _expectedLen);
    }
    _bufLen = 0;
}

void ArdufliteCRSFReceiver::dispatchFrame(const uint8_t* frame, size_t len) 
{
    uint8_t type = frame[2];
    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) 
    {
        // got a real RC frame: reset failsafe timer
        {
            SemaphoreLock lock(_lock);
            _lastRcMicros = micros();
            if (_inFailsafe) 
            {
                LOG_INF("Exiting RC failsafe");
                _inFailsafe = false;
            }
        }
        
        {
            SemaphoreLock lock(_lock);
            handleRC(frame + 3, len - 4);
        }

    } 
    else if (type == CRSF_FRAMETYPE_LINK_STATISTICS) 
    {
        crsfLinkStatistics_t stats;
        std::memcpy(&stats, frame + 3, sizeof(stats));

        {
            SemaphoreLock lock(_lock);
            _latestLinkStats = stats;
            _haveLinkStats   = true;
        }
    } 
    else if (type == CRSF_FRAMETYPE_DEVICE_PING) 
    {
        // you could reply or log ping timestamps here…
    } 
    else if (type == CRSF_FRAMETYPE_DEVICE_INFO) 
    {
        // parse device-info payload if you like
    } 
    else if (type == CRSF_FRAMETYPE_SUBSCRIBE_TELEMETRY) 
    {
        // the TX telling you which telemetry it wants
    }
}

void ArdufliteCRSFReceiver::handleRC(const uint8_t* payload, size_t) 
{
    uint16_t raw[16];
    decodeChannels(payload, raw);
    for (uint8_t ch = 0; ch < 16; ++ch) 
    {
        // only fire when the raw value changed
        if (_chCfg[ch].callback && raw[ch] != _lastRaw[ch]) 
        {
            _lastRaw[ch] = raw[ch];
            float v = applyMapping(_chCfg[ch], raw[ch]);
            _chCfg[ch].callback(ch, v);
        }
    }
}

void ArdufliteCRSFReceiver::decodeChannels(const uint8_t* p, uint16_t out[16]) 
{
    for (uint8_t i = 0; i < 16; ++i) 
    {
        uint32_t bit = i*11;
        uint32_t byte = bit/8;
        uint32_t off = bit%8;
        uint32_t val = p[byte] >> off;
        val |= uint32_t(p[byte+1]) << (8-off);
        if (off > 5) val |= uint32_t(p[byte+2]) << (16-off);
        out[i] = val & 0x07FF;
    }
}

float ArdufliteCRSFReceiver::applyMapping(const ChannelConfig& c, uint16_t r) 
{
    switch (c.type) 
    {
        case ChannelType::Raw:
            return (float)r;
        case ChannelType::DualThrow:
            return ((float)r - 1024.0f)/1023.0f;
        case ChannelType::SingleThrow:
            return (float)r/2047.0f;
        case ChannelType::Boolean:
            return r > 1024 ? 1.0f : 0.0f;
        case ChannelType::TriState: 
        {
            float n = (float)r/2047.0f;
            if (n < c.thrLow)   return -1.0f;
            if (n > c.thrHigh)  return +1.0f;
            return 0.0f;
        }
        case ChannelType::Custom:
            return c.converter ? c.converter(r) : (float)r;
    }
    return 0.0f;
}
