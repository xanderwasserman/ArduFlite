// Colors.h
#pragma once
#include <cstdint>

#include "src/hal/device/Peripherals.h"

/// The palette speaks the HAL's colour type directly, so nothing converts
/// between two identical RGB structs on the way to the indicator.
using Color = arduflite::device::Rgb;

/// A small palette of named colours.
namespace Colors {
    constexpr Color Red    { 255,   0,   0 };
    constexpr Color Green  {   0, 255,   0 };
    constexpr Color Blue   {   0,   0, 255 };
    constexpr Color Yellow { 255, 255,   0 };
    constexpr Color Orange { 255, 165,   0 };
    constexpr Color White  { 255, 255, 255 };
    constexpr Color Black  {   0,   0,   0 };
    // …add more as necessary
}

using Pattern = arduflite::device::BlinkPattern;

namespace Patterns {

    /// Solid yellow (on boot)
    constexpr Pattern Boot  { Colors::Yellow, 0, 0 };

    /// Fast red blink for errors
    constexpr Pattern Error { Colors::Red, 100, 100 };

    /// Slow green blink for “all good” in Assist Mode
    constexpr Pattern Assist    { Colors::Green, 500, 500 };

    /// Slow white blink for “all good” in Stabilised Mode
    constexpr Pattern Stabilized    { Colors::White, 500, 500 };

    
}