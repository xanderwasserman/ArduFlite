/**
 * Watchdog.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_WATCHDOG_H
#define ARDUFLITE_HAL_PLATFORM_WATCHDOG_H

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"

namespace arduflite::hal {

class Watchdog : private NonCopyable
{
public:
    virtual ~Watchdog() = default;

    virtual Status registerCurrentTask()   = 0;
    virtual void   feed() noexcept         = 0;
    virtual Status unregisterCurrentTask() = 0;
};

/**
 * @brief RAII registration, replacing manual esp_task_wdt_add/_delete pairs
 *        scattered across five task functions.
 */
class WatchdogGuard : private NonCopyable
{
public:
    explicit WatchdogGuard(Watchdog& wd) : _wd(wd) { (void)_wd.registerCurrentTask(); }
    ~WatchdogGuard()                               { (void)_wd.unregisterCurrentTask(); }

    void feed() noexcept { _wd.feed(); }

private:
    Watchdog& _wd;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_WATCHDOG_H
