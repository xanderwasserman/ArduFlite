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
  #include <cstdio>    // for printf in tests/simulation
#endif

#include <Arduino.h>  // for Serial.printf on-device

// Default handler: sends to Serial.printf()
class SerialLogHandler : public LogHandler {
public:
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
        Serial.printf("%s", tag);
        Serial.vprintf(fmt, args);
    }

    void log_nl(LogLevel level, const char* fmt, va_list args) override {
        log(level, fmt, args);
        Serial.printf("\r\n");
    }
};

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
  : _level(LogLevel::Debug)
{
    // default handler is Serial
    static SerialLogHandler serialHandler;
    _handler = &serialHandler;
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
