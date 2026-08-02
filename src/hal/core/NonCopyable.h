/**
 * NonCopyable.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Base for every HAL interface. Prevents accidental slicing of a driver
 *        into its interface, which is a silent and hard-to-find bug.
 */
#ifndef ARDUFLITE_HAL_CORE_NONCOPYABLE_H
#define ARDUFLITE_HAL_CORE_NONCOPYABLE_H

namespace arduflite {

class NonCopyable
{
protected:
    constexpr NonCopyable() noexcept = default;
    ~NonCopyable()                   = default;   // non-virtual: never delete through this

public:
    NonCopyable(const NonCopyable&)            = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
    NonCopyable(NonCopyable&&)                 = delete;
    NonCopyable& operator=(NonCopyable&&)      = delete;
};

} // namespace arduflite

#endif // ARDUFLITE_HAL_CORE_NONCOPYABLE_H
