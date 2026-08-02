/**
 * Status.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Value-returned error type for the HAL.
 *
 * Marked [[nodiscard]] on the enum itself (C++20), so EVERY function returning
 * Status warns if its result is dropped. See specs/hal/07-decisions.md ADR-009.
 */
#ifndef ARDUFLITE_HAL_CORE_STATUS_H
#define ARDUFLITE_HAL_CORE_STATUS_H

#include <cstdint>

namespace arduflite {

enum class [[nodiscard]] Status : std::uint8_t
{
    Ok = 0,
    NotPresent,      ///< Device did not acknowledge — not fitted
    IoError,         ///< Bus transaction failed
    Timeout,
    InvalidArg,
    NotSupported,    ///< Driver does not implement this capability
    NotInitialised,  ///< begin() not called, or it failed
    Busy,            ///< Resource held by another owner
    OutOfRange,
    Corrupt,         ///< CRC / magic / schema mismatch
    NoSpace,
};

/// Human-readable name, for logs and the CLI. Never returns nullptr.
constexpr const char* toString(Status s) noexcept
{
    switch (s)
    {
        case Status::Ok:             return "Ok";
        case Status::NotPresent:     return "NotPresent";
        case Status::IoError:        return "IoError";
        case Status::Timeout:        return "Timeout";
        case Status::InvalidArg:     return "InvalidArg";
        case Status::NotSupported:   return "NotSupported";
        case Status::NotInitialised: return "NotInitialised";
        case Status::Busy:           return "Busy";
        case Status::OutOfRange:     return "OutOfRange";
        case Status::Corrupt:        return "Corrupt";
        case Status::NoSpace:        return "NoSpace";
    }
    return "Unknown";
}

constexpr bool isOk(Status s) noexcept { return s == Status::Ok; }

} // namespace arduflite

#endif // ARDUFLITE_HAL_CORE_STATUS_H
