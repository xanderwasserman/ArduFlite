/**
 * Arduino.h — minimal stub for host-side unit tests.
 *
 * Provides only the symbols used by the sources under test (pid.cpp, etc.)
 * so they compile on a standard C++ host without the Arduino SDK.
 */
#pragma once

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
#endif

// ── Timing stubs (not exercised in logic under test) ──────────────────────
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
