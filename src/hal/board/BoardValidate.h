/**
 * BoardValidate.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Compile-time board validation.
 *
 * Applied to the legacy BOARD_TYPE_WEMOS pin table, these turn four real defects
 * (see specs/hal/00-current-state.md 2.4) into build errors:
 *   - GPIO 32 assigned on a chip whose GPIOs stop at 21
 *   - throttle input and throttle output both on GPIO 10
 *   - roll input and CRSF RX both on GPIO 6
 *   - yaw input and CRSF TX both on GPIO 8
 *
 * Predicates are constexpr so they can be unit-tested directly; isValid() is
 * consteval so a static_assert cannot silently fall back to a runtime check.
 */
#ifndef ARDUFLITE_HAL_BOARD_BOARDVALIDATE_H
#define ARDUFLITE_HAL_BOARD_BOARDVALIDATE_H

#include <string_view>

#include "src/hal/board/BoardDescriptor.h"

namespace arduflite::board::validate {

// ── Single-pin predicates ───────────────────────────────────────────────────

constexpr bool inRange(const McuProfile& m, Pin p) noexcept
{
    return p == kNoPin || (p >= m.minGpio && p <= m.maxGpio);
}

/// @note The width guard is not paranoia: shifting by >= the operand width is
///       undefined behaviour, and an out-of-range pin would otherwise reach it.
///       A pin too large to be represented is treated as set, i.e. unusable.
constexpr bool maskBitSet(std::uint64_t mask, Pin p) noexcept
{
    if (p < 0)  { return false; }
    if (p >= 64) { return true; }
    return ((mask >> static_cast<unsigned>(p)) & 1ull) != 0ull;
}

constexpr bool notReserved(const McuProfile& m, Pin p) noexcept
{
    return p == kNoPin || !maskBitSet(m.reservedMask, p);
}

constexpr bool canOutput(const McuProfile& m, Pin p) noexcept
{
    return p == kNoPin || !maskBitSet(m.inputOnlyMask, p);
}

// ── Pin collection ──────────────────────────────────────────────────────────

/// Every pin the firmware will touch, gathered so uniqueness can be checked.
struct PinList
{
    static constexpr int kMax = 32;

    Pin pins[kMax]{};
    int count = 0;

    constexpr void add(Pin p) noexcept
    {
        if (p != kNoPin && count < kMax) { pins[count++] = p; }
    }
};

constexpr PinList collectPins(const BoardDescriptor& b) noexcept
{
    PinList list;
    list.add(b.sensorBus.sda);
    list.add(b.sensorBus.scl);
    list.add(b.rcUart.rx);
    list.add(b.rcUart.tx);
    list.add(b.consoleUart.rx);
    list.add(b.consoleUart.tx);
    list.add(b.userButton.pin);
    list.add(b.statusLed.pin);

    for (std::uint8_t i = 0; i < b.bankCount; ++i)
    {
        const auto& bank = b.actuatorBanks[i];
        for (std::uint8_t j = 0; j < bank.outputCount; ++j)
        {
            list.add(bank.outputs[j].pin);
        }
    }
    return list;
}

// ── Board-level predicates ──────────────────────────────────────────────────

constexpr bool allPinsValid(const BoardDescriptor& b) noexcept
{
    const PinList list = collectPins(b);
    for (int i = 0; i < list.count; ++i)
    {
        if (!inRange(b.mcu, list.pins[i]))     { return false; }
        if (!notReserved(b.mcu, list.pins[i])) { return false; }
    }
    return true;
}

constexpr bool allPinsUnique(const BoardDescriptor& b) noexcept
{
    const PinList list = collectPins(b);
    for (int i = 0; i < list.count; ++i)
    {
        for (int j = i + 1; j < list.count; ++j)
        {
            if (list.pins[i] == list.pins[j]) { return false; }
        }
    }
    return true;
}

constexpr bool allOutputPinsCanOutput(const BoardDescriptor& b) noexcept
{
    for (std::uint8_t i = 0; i < b.bankCount; ++i)
    {
        const auto& bank = b.actuatorBanks[i];
        for (std::uint8_t j = 0; j < bank.outputCount; ++j)
        {
            if (!canOutput(b.mcu, bank.outputs[j].pin)) { return false; }
        }
    }
    return canOutput(b.mcu, b.statusLed.pin);
}

constexpr bool outputCountWithinMcu(const BoardDescriptor& b) noexcept
{
    int pwmOutputs = 0;
    for (std::uint8_t i = 0; i < b.bankCount; ++i)
    {
        const auto& bank = b.actuatorBanks[i];
        if (bank.transport == ActuatorTransport::Pwm ||
            bank.transport == ActuatorTransport::DShot)
        {
            pwmOutputs += bank.outputCount;
        }
    }
    return pwmOutputs <= static_cast<int>(b.mcu.pwmChannelCount);
}

/// Roles are how flight code resolves actuators, so a duplicate would make
/// byRole() silently ambiguous.
constexpr bool allRolesUnique(const BoardDescriptor& b) noexcept
{
    for (std::uint8_t bi = 0; bi < b.bankCount; ++bi)
    {
        for (std::uint8_t oi = 0; oi < b.actuatorBanks[bi].outputCount; ++oi)
        {
            const std::string_view a{ b.actuatorBanks[bi].outputs[oi].role };
            if (a.empty()) { return false; }

            for (std::uint8_t bj = bi; bj < b.bankCount; ++bj)
            {
                const std::uint8_t start = (bj == bi) ? static_cast<std::uint8_t>(oi + 1) : 0;
                for (std::uint8_t oj = start; oj < b.actuatorBanks[bj].outputCount; ++oj)
                {
                    if (a == std::string_view{ b.actuatorBanks[bj].outputs[oj].role })
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

/// Every sensor's axis map must be a real permutation.
constexpr bool allSensorAxesValid(const BoardDescriptor& b) noexcept
{
    for (std::uint8_t i = 0; i < b.sensorCount; ++i)
    {
        if (!b.sensors[i].axes.isValid()) { return false; }
    }
    return true;
}

/// "Every fitted part has a bus and pins." Skipped for Untested boards.
constexpr bool requiredPeripheralsPresent(const BoardDescriptor& b) noexcept
{
    if (b.maturity == BoardMaturity::Untested) { return true; }

    if (b.rcLink == RcPart::Crsf)
    {
        if (b.rcUart.rx == kNoPin || b.rcUart.tx == kNoPin) { return false; }
    }
    if (b.rcLink == RcPart::Unknown) { return false; }

    for (std::uint8_t i = 0; i < b.sensorCount; ++i)
    {
        if (b.sensors[i].bus == BusKind::I2c &&
            (b.sensorBus.sda == kNoPin || b.sensorBus.scl == kNoPin))
        {
            return false;
        }
    }
    return true;
}

constexpr bool isValidConstexpr(const BoardDescriptor& b) noexcept
{
    return allPinsValid(b)
        && allPinsUnique(b)
        && allOutputPinsCanOutput(b)
        && outputCountWithinMcu(b)
        && allRolesUnique(b)
        && allSensorAxesValid(b)
        && requiredPeripheralsPresent(b);
}

/// consteval: a static_assert on this CANNOT silently degrade to a runtime check.
consteval bool isValid(const BoardDescriptor& b) noexcept
{
    return isValidConstexpr(b);
}

} // namespace arduflite::board::validate

#endif // ARDUFLITE_HAL_BOARD_BOARDVALIDATE_H
