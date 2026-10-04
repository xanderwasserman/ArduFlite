/**
 * Arduino.h — minimal stub for host-side unit tests.
 *
 * Provides only the symbols used by the sources under test (pid.cpp, etc.)
 * so they compile on a standard C++ host without the Arduino SDK.
 */
#pragma once

#include <string>

#include <cmath>
#include <cstdint>
#include <algorithm>

// ── Arduino constrain macro ────────────────────────────────────────────────
#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// ── Math constants (not guaranteed by all stdlib implementations) ──────────
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arduino-style PI alias
#ifndef PI
#define PI static_cast<float>(M_PI)

/**
 * @brief Minimal stand-in for Arduino's String.
 *
 * Enough for ConfigRegistry's key storage, which is the only thing the host
 * builds reach. Not a faithful reimplementation and not trying to be — if
 * something starts needing more of Arduino's String API on a host, that is a
 * signal to get String out of that interface rather than to grow this.
 */
class String : public std::string
{
public:
    String() = default;
    String(const char* s) : std::string(s ? s : "") {}
    String(const std::string& s) : std::string(s) {}

    [[nodiscard]] const char* c_str() const { return std::string::c_str(); }
    [[nodiscard]] unsigned length() const { return (unsigned)std::string::size(); }
};


/// Arduino trig constants.
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif

/**
 * @brief Opaque FreeRTOS handle, purely so headers that DECLARE one parse.
 *
 * ConfigRegistry still holds a raw SemaphoreHandle_t. host_sim includes its
 * header (the controllers call initFromConfig on the target) but never calls
 * into it, so an opaque pointer is enough to compile.
 *
 * This is a shim for a real coupling, not a fix for it. ConfigRegistry has not
 * been migrated to hal::Mutex; if host_sim ever needs to READ config, migrate
 * it rather than growing this.
 */
using SemaphoreHandle_t = void*;

#endif

// ── Timing stubs (not exercised in logic under test) ──────────────────────
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
