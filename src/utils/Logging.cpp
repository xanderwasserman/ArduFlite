/**
 * Logging.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 27 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#include "src/utils/Logging.h"

#ifdef ARDUFLITE_UNIT_TEST
  #include <cstdio>
#include <cstring>    // for printf in tests/simulation
#endif

#include <cstdio>
#include <cstring>

#ifndef ARDUFLITE_UNIT_TEST
#include "src/hal/board/Board.h"
#endif

// Default handler: sends to Serial.printf()
/**
 * @brief Writes log lines to a device::Console.
 *
 * The console is INJECTED rather than fetched from Board inside each call. It
 * was written the other way first, which made this class untestable on a host
 * and hid a dependency that the type never mentioned — the same reach-for-a-
 * global pattern the HAL exists to remove (§01, the AP_HAL lesson).
 */
#ifndef ARDUFLITE_UNIT_TEST
class ConsoleLogHandler : public LogHandler {
public:
    explicit ConsoleLogHandler(arduflite::device::Console& console) : _console(console) {}

    void log(LogLevel level, const char* fmt, va_list args) override {
        // Prepend a level tag
        const char* tag = "";
        switch(level) {
            case LogLevel::Debug: tag = "[DEBUG] "; break;
            case LogLevel::Info:  tag = "[INFO] "; break;
            case LogLevel::Warn:  tag = "[WARN] "; break;
            case LogLevel::Error: tag = "[ERROR] "; break;
            case LogLevel::Clear: tag = ""; break;
            default: break;
        }
        // Formatted into a stack buffer, then handed to the console as bytes.
        // vsnprintf rather than the port's own vprintf so this handler depends
        // on device::Console alone — the same reason the buffer is bounded
        // rather than relying on the port to truncate.
        _console.write(tag, std::strlen(tag));

        char buffer[kMaxLine];
        const int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
        if (written > 0)
        {
            const std::size_t length =
                (static_cast<std::size_t>(written) < sizeof(buffer))
                    ? static_cast<std::size_t>(written)
                    : sizeof(buffer) - 1;
            _console.write(buffer, length);
        }
    }

    void log_nl(LogLevel level, const char* fmt, va_list args) override {
        log(level, fmt, args);

        _console.write("\r\n", 2);

        // Errors are flushed immediately. The port is USB CDC on the C3, so a
        // watchdog reset or panic can discard buffered output — and the line
        // most worth keeping is the one written just before the reset.
        // Non-error levels are left buffered: flushing at 500 Hz would stall
        // the writing task on USB.
        if (level == LogLevel::Error) { _console.flushOutput(); }
    }

    /// Longest single log line. Anything beyond this is truncated rather than
    /// overflowing; log lines are diagnostics, not a transport.
    static constexpr std::size_t kMaxLine = 256;

private:
    arduflite::device::Console& _console;
};
#endif // !ARDUFLITE_UNIT_TEST

// Optional stdout handler (for unit tests / simulation)
#ifdef ARDUFLITE_UNIT_TEST
class StdoutLogHandler : public LogHandler {
public:
    void log(LogLevel level, const char* fmt, va_list args) override {
        const char* tag = "";
        switch(level) {
            case LogLevel::Debug: tag = "[D] "; break;
            case LogLevel::Info:  tag = "[I] "; break;
            case LogLevel::Warn:  tag = "[W] "; break;
            case LogLevel::Error: tag = "[E] "; break;
            default: break;
        }
        printf("%s", tag);
        vprintf(fmt, args);
    }

    void log_nl(LogLevel level, const char* fmt, va_list args) override {
        log(level, fmt, args);
        printf("\n");
    }
};
#endif

// --- Logger implementation ---

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::Logger()
  : _level(LogLevel::Info)
{
    // default handler is Serial
#ifdef ARDUFLITE_UNIT_TEST
    // Host builds have no Board and no console. StdoutLogHandler existed for
    // this but was never selected — instance() reached for Board regardless,
    // which is why Logging.cpp could not be linked into the unit tests.
    static StdoutLogHandler stdoutHandler;
    _handler = &stdoutHandler;
#else
    // Constructed on first use, after Board::begin() has opened the console.
    static ConsoleLogHandler consoleHandler{ arduflite::board::Board::instance().console() };
    _handler = &consoleHandler;
#endif
}

Logger::~Logger() {
    // nothing
}

void Logger::setLevel(LogLevel level) {
    _level = level;
}

void Logger::setHandler(LogHandler* handler) {
    if (handler) _handler = handler;
}

void Logger::vprint(LogLevel lvl, const char* fmt, va_list args) {
    if (lvl < _level || !_handler) return;
    _handler->log_nl(lvl, fmt, args);
}

void Logger::vprint_n(LogLevel lvl, const char* fmt, va_list args) {
    if (lvl < _level || !_handler) return;
    _handler->log(lvl, fmt, args);
}

// Newline logging functions
void Logger::debug(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint(LogLevel::Debug, fmt, args);
    va_end(args);
}

void Logger::info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint(LogLevel::Info, fmt, args);
    va_end(args);
}

void Logger::warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint(LogLevel::Warn, fmt, args);
    va_end(args);
}

void Logger::error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint(LogLevel::Error, fmt, args);
    va_end(args);
}

void Logger::clear(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint(LogLevel::Clear, fmt, args);
    va_end(args);
}

// Non-newline logging functions
void Logger::debug_n(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint_n(LogLevel::Debug, fmt, args);
    va_end(args);
}

void Logger::info_n(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint_n(LogLevel::Info, fmt, args);
    va_end(args);
}

void Logger::warn_n(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint_n(LogLevel::Warn, fmt, args);
    va_end(args);
}

void Logger::error_n(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint_n(LogLevel::Error, fmt, args);
    va_end(args);
}

void Logger::clear_n(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint_n(LogLevel::Clear, fmt, args);
    va_end(args);
}
