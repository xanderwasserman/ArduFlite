/**
 * Logging.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 27 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#ifndef ARDUFLITE_LOGGING_H
#define ARDUFLITE_LOGGING_H

#include <cstdarg>

// prepend the caller’s function name to every format string
#define LOG_DBG(fmt, ...)  Logger::instance().debug("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_INF(fmt, ...)  Logger::instance().info ("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) Logger::instance().warn ("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)  Logger::instance().error("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_C(fmt, ...)    Logger::instance().clear("[%s] " fmt, __func__, ##__VA_ARGS__)

// and the “no-newline” variants:
#define LOG_DBG_N(fmt, ...)  Logger::instance().debug_n("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_INF_N(fmt, ...)  Logger::instance().info_n ("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_WARN_N(fmt, ...) Logger::instance().warn_n ("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_ERR_N(fmt, ...)  Logger::instance().error_n("[%s] " fmt, __func__, ##__VA_ARGS__)
#define LOG_C_N(fmt, ...)    Logger::instance().clear_n("[%s] " fmt, __func__, ##__VA_ARGS__)

// Log levels
enum class LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Clear,
    Off
};

// Abstract interface for log output
class LogHandler {
public:
    virtual ~LogHandler() {}
    virtual void log(LogLevel level, const char* fmt, va_list args) = 0;
    virtual void log_nl(LogLevel level, const char* fmt, va_list args) = 0;
};

// The Logger singleton
class Logger {
public:
    // Get the single instance
    static Logger& instance();

    // Set the minimum level to actually emit
    void setLevel(LogLevel level);

    // Swap in a different handler (e.g. for testing)
    void setHandler(LogHandler* handler);

    // Logging methods
    void debug(const char* fmt, ...);
    void info (const char* fmt, ...);
    void warn (const char* fmt, ...);
    void error(const char* fmt, ...);
    void clear(const char* fmt, ...);

    // Logging methods without newlines
    void debug_n(const char* fmt, ...);
    void info_n(const char* fmt, ...);
    void warn_n(const char* fmt, ...);
    void error_n(const char* fmt, ...);
    void clear_n(const char* fmt, ...);

private:
    Logger();
    ~Logger();

    LogLevel _level;
    LogHandler* _handler;

    void vprint(LogLevel lvl, const char* fmt, va_list args);
    void vprint_n(LogLevel lvl, const char* fmt, va_list args);
};

#endif // ARDUFLITE_LOGGING_H
