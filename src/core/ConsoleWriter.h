/**
 * ConsoleWriter.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief printf-style writing straight to a device::Console.
 *
 * For DATA, not diagnostics. The serial telemetry backends emit
 * through LOG()/LOG_N(), which put their output behind the diagnostic logger's
 * level filter and interleaved it with log lines — so `log off` silently
 * stopped telemetry, and an error logged mid-flight landed in the middle of a
 * CSV row that a plotter was parsing (ADR-044).
 *
 * Diagnostics and data are different channels that happen to share a wire. This
 * is the data one: no level, no tag, no filtering.
 */
#ifndef ARDUFLITE_CORE_CONSOLE_WRITER_H
#define ARDUFLITE_CORE_CONSOLE_WRITER_H

#include <cstdarg>
#include <cstdio>

#include "src/hal/device/Peripherals.h"

namespace arduflite {

class ConsoleWriter
{
public:
    explicit ConsoleWriter(device::Console& console) : _console(console) {}

    /// Writes exactly what it is given. No newline is appended — callers that
    /// want one put it in the format string, as the logging macros did.
    void printf(const char* fmt, ...)
    {
        char buffer[kMaxLine];

        va_list args;
        va_start(args, fmt);
        const int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);

        if (written <= 0) { return; }

        const std::size_t length = (static_cast<std::size_t>(written) < sizeof(buffer))
                                       ? static_cast<std::size_t>(written)
                                       : sizeof(buffer) - 1;
        _console.write(buffer, length);
    }

private:
    /// One telemetry row. Bounded rather than dynamic — this runs in a task at
    /// up to 50 Hz and must not allocate.
    static constexpr std::size_t kMaxLine = 256;

    device::Console& _console;
};

} // namespace arduflite

#endif // ARDUFLITE_CORE_CONSOLE_WRITER_H
