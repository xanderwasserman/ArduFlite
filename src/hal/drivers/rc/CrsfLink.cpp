/**
 * CrsfLink.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/rc/CrsfLink.h"

namespace arduflite::drivers {

Status CrsfLink::begin()
{
    _parser.reset();
    _haveFrame.store(false, std::memory_order_release);
    _inFailsafe.store(false, std::memory_order_release);
    _everReceived = false;
    return Status::Ok;
}

void CrsfLink::poll()
{
    std::uint8_t buf[64];
    while (_uart.available() > 0)
    {
        const std::size_t n = _uart.read(buf, sizeof(buf));
        if (n == 0) { break; }

        for (std::size_t i = 0; i < n; ++i)
        {
            switch (_parser.feed(buf[i]))
            {
                case CrsfParser::Event::RcChannels:
                {
                    const std::uint16_t* raw = _parser.channels();
                    for (std::uint8_t c = 0; c < device::RcFrame::kMaxChannels; ++c)
                    {
                        _channelsUs[c] = rawToMicroseconds(raw[c]);
                    }
                    _lastRcFrame  = _clock.now();
                    _everReceived = true;
                    _haveFrame.store(true, std::memory_order_release);

                    // Recovery edge. Entering failsafe is decided by the timeout
                    // below; leaving it is decided by a frame arriving.
                    if (_inFailsafe.exchange(false, std::memory_order_acq_rel))
                    {
                        // Edge consumed; RcMapper/flight layer observes via isFailsafe().
                    }
                    break;
                }

                case CrsfParser::Event::LinkStatistics:
                {
                    const auto& s = _parser.linkStats();
                    // CRSF sends RSSI as (dBm + 64); recover the signed dBm.
                    const auto toDbm = [](std::uint8_t v) {
                        return static_cast<std::int8_t>(static_cast<int>(v) - 64);
                    };

                    _stats.uplinkQuality_pct   = s.uplinkLinkQuality;
                    _stats.uplinkRssi1_dbm     = toDbm(s.uplinkRssi1);
                    _stats.uplinkRssi2_dbm     = toDbm(s.uplinkRssi2);
                    _stats.uplinkSnr_db        = s.uplinkSnr;
                    _stats.downlinkQuality_pct = s.downlinkLinkQuality;
                    _stats.downlinkRssi_dbm    = toDbm(s.downlinkRssi);
                    _stats.downlinkSnr_db      = s.downlinkSnr;
                    _stats.activeAntenna       = s.activeAntenna;
                    _stats.rfMode              = s.rfMode;
                    _stats.txPower             = s.uplinkTxPower;
                    _stats.valid               = true;
                    _haveStats.store(true, std::memory_order_release);
                    break;
                }

                default:
                    break;
            }
        }
    }

    // Failsafe entry: no RC frame within the timeout. Only armed once a first
    // frame has been seen, so a receiver that never connects does not look like
    // a link that dropped.
    if (_everReceived)
    {
        const auto since = _clock.now() - _lastRcFrame;
        if (since > _failsafeTimeout)
        {
            _inFailsafe.store(true, std::memory_order_release);
        }
    }
}

bool CrsfLink::readFrame(device::RcFrame& out)
{
    if (!_haveFrame.exchange(false, std::memory_order_acq_rel)) { return false; }

    for (std::uint8_t c = 0; c < device::RcFrame::kMaxChannels; ++c)
    {
        out.channel_us[c] = _channelsUs[c];
    }
    out.channelCount = device::RcFrame::kMaxChannels;
    out.time         = _lastRcFrame;
    return true;
}

device::RcLinkStats CrsfLink::stats() const
{
    device::RcLinkStats s = _stats;
    s.valid = _haveStats.load(std::memory_order_acquire);
    return s;
}

} // namespace arduflite::drivers
