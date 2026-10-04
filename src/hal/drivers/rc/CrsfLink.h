/**
 * CrsfLink.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief device::RcLink over CRSF.
 *
 * Owns a UART, a CrsfParser and a failsafe timer. It does NOT know that channel
 * 5 is ARM — role mapping lives in input::RcMapper (ADR-006). That separation is
 * what lets a PWM or SBUS link substitute without touching flight code.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_RC_CRSFLINK_H
#define ARDUFLITE_HAL_DRIVERS_RC_CRSFLINK_H

#include <atomic>

#include "src/hal/device/RcLink.h"
#include "src/hal/drivers/rc/CrsfParser.h"
#include "src/hal/platform/Buses.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::drivers {

class CrsfLink final : public device::RcLink
{
public:
    CrsfLink(hal::Uart& uart, const hal::Clock& clock) : _uart(uart), _clock(clock) {}

    Status begin() override;

    [[nodiscard]] bool readFrame(device::RcFrame& out) override;
    [[nodiscard]] bool isFailsafe() const override
    {
        return _inFailsafe.load(std::memory_order_acquire);
    }
    void setFailsafeTimeout(std::chrono::milliseconds timeout) override
    {
        _failsafeTimeout = timeout;
    }

    [[nodiscard]] device::RcLinkStats stats() const override;
    [[nodiscard]] const char* name() const override { return "CRSF"; }

    /// Drain the UART and update failsafe state. Called by the RC task.
    void poll();

    /// Parser diagnostics — a rising CRC error count is the first sign of a
    /// marginal link, and there was no way to see it before.
    [[nodiscard]] std::uint32_t crcErrors() const noexcept { return _parser.crcErrors(); }

    /// CRSF/ELRS channel endpoints. These are the protocol's, not this
    /// project's: a transmitter at -100% sends 172 and at +100% sends 1811,
    /// with 992 as centre. The 11-bit field can carry 0..2047, but a radio
    /// only reaches the extremes with >100% travel configured.
    static constexpr std::uint16_t kRawMin    = 172;
    static constexpr std::uint16_t kRawCentre = 992;
    static constexpr std::uint16_t kRawMax    = 1811;

    /**
     * @brief CRSF 11-bit raw value to microseconds.
     *
     *   raw  172 -> 1000 us      normalised -1
     *   raw  992 -> 1500 us      normalised  0
     *   raw 1811 -> 2000 us      normalised +1
     *
     * Raw values outside [172, 1811] are clamped, so a radio set beyond 100%
     * travel reaches the endpoint and stops rather than driving the servo past
     * it. RcMapper::toBipolar() clamps to +-1 again on the way out.
     *
     * The round trip costs resolution: 1640 usable raw steps become 1001
     * microsecond steps, ~0.1% of full travel — below what a pilot can command
     * or an airframe responds to, and it buys interchangeability with PWM and
     * SBUS links.
     */
    [[nodiscard]] static constexpr std::uint16_t rawToMicroseconds(std::uint16_t raw) noexcept
    {
        if (raw < kRawMin) { raw = kRawMin; }
        if (raw > kRawMax) { raw = kRawMax; }

        constexpr std::uint32_t span = kRawMax - kRawMin;   // 1639
        const std::uint32_t offset = static_cast<std::uint32_t>(raw) - kRawMin;
        return static_cast<std::uint16_t>(1000u + (offset * 1000u + span / 2u) / span);
    }

private:
    hal::Uart&        _uart;
    const hal::Clock& _clock;
    CrsfParser        _parser{};

    std::uint16_t          _channelsUs[device::RcFrame::kMaxChannels]{};
    std::atomic<bool>      _haveFrame{ false };
    std::atomic<bool>      _inFailsafe{ false };
    hal::Clock::time_point _lastRcFrame{};
    bool                   _everReceived = false;

    std::chrono::milliseconds _failsafeTimeout{ 500 };

    device::RcLinkStats _stats{};
    std::atomic<bool>   _haveStats{ false };
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_RC_CRSFLINK_H
