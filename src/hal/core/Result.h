/**
 * Result.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Result<T> — a value or a Status. No exceptions, no heap.
 *
 * std::expected would be the natural fit but is C++23; the ESP32 core resolves
 * to C++20 (-std=gnu++2a). See specs/hal/09-cpp-conventions.md.
 */
#ifndef ARDUFLITE_HAL_CORE_RESULT_H
#define ARDUFLITE_HAL_CORE_RESULT_H

#include <type_traits>
#include <utility>

#include "src/hal/core/Status.h"

namespace arduflite {

template <typename T>
class [[nodiscard]] Result
{
public:
    constexpr Result(T value) noexcept(std::is_nothrow_move_constructible_v<T>)
        : _value(std::move(value)), _status(Status::Ok) {}

    constexpr Result(Status error) noexcept : _value{}, _status(error) {}

    [[nodiscard]] constexpr bool   ok()     const noexcept { return _status == Status::Ok; }
    [[nodiscard]] constexpr Status status() const noexcept { return _status; }

    constexpr explicit operator bool() const noexcept { return ok(); }

    /// @note Precondition: ok(). Ref-qualified so a temporary moves out.
    [[nodiscard]] constexpr const T& value() const&  noexcept { return _value; }
    [[nodiscard]] constexpr T&&      value()      && noexcept { return std::move(_value); }

    template <typename U>
    [[nodiscard]] constexpr T valueOr(U&& fallback) const&
    {
        return ok() ? _value : static_cast<T>(std::forward<U>(fallback));
    }

private:
    T      _value{};
    Status _status;
};

} // namespace arduflite

/// Early-return on failure, preserving the status.
/// @warning Never use inside a loop that must keep going — see specs/hal review R8.
#define ARDUFLITE_TRY(expr)                                        \
    do {                                                           \
        const ::arduflite::Status _arduflite_s = (expr);           \
        if (_arduflite_s != ::arduflite::Status::Ok)               \
        {                                                          \
            return _arduflite_s;                                   \
        }                                                          \
    } while (false)

#endif // ARDUFLITE_HAL_CORE_RESULT_H
