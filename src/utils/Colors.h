// Colors.h
#pragma once
#include <cstdint>

/// Simple RGB container.
struct Color {
    uint8_t r, g, b;
};

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

/// A blink pattern: { r, g, b, on_ms, off_ms }
struct Pattern { uint8_t r,g,b; uint16_t on_ms, off_ms; };

namespace Patterns {

    /// Solid yellow (on boot)
    constexpr Pattern Boot  { Colors::Yellow.r, Colors::Yellow.g, Colors::Yellow.b,    0,   0   };

    /// Fast red blink for errors
    constexpr Pattern Error { Colors::Red.r,    Colors::Red.g,    Colors::Red.b, 100, 100 };

    /// Slow green blink for “all good” in Assist Mode
    constexpr Pattern Assist    { Colors::Green.r,  Colors::Green.g,  Colors::Green.b, 500, 500 };

    /// Slow white blink for “all good” in Stabilised Mode
    constexpr Pattern Stabilized    { Colors::White.r,  Colors::White.g,  Colors::White.b, 500, 500 };

    
}