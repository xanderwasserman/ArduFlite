/**
 * HostStubs.cpp — host_sim
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Stand-ins for the two things the control loops still reach for.
 *
 * These exist because `initFromConfig()` couples both inner controllers to
 * ConfigRegistry, and every module logs through Logger — and BOTH of those are
 * still Arduino/FreeRTOS coupled. So the controllers are portable, but their
 * configuration path is not.
 *
 * That is a real finding, not an inconvenience: see §06 Phase 8. Stubbing them
 * here keeps host_sim honest about the boundary rather than hiding it — the
 * loops run on tuning constants supplied below, NOT on the aircraft's
 * configuration, and this file is where that difference lives.
 */
#include <cstdarg>
#include <cstdio>

#include "src/utils/ConfigRegistry.h"
#include "src/utils/Logging.h"

// ── Logger ──────────────────────────────────────────────────────────────────

Logger::Logger()  = default;
Logger::~Logger() = default;

Logger& Logger::instance()
{
    static Logger logger;
    return logger;
}

namespace {

void emit(const char* tag, const char* fmt, va_list args)
{
    printf("%s", tag);
    vprintf(fmt, args);
    printf("\n");
}

} // namespace

#define ARDUFLITE_SIM_LOG(method, tag)                       \
    void Logger::method(const char* fmt, ...)                \
    {                                                        \
        va_list args; va_start(args, fmt);                   \
        emit(tag, fmt, args);                                \
        va_end(args);                                        \
    }

ARDUFLITE_SIM_LOG(debug, "[DEBUG] ")
ARDUFLITE_SIM_LOG(info,  "[INFO]  ")
ARDUFLITE_SIM_LOG(warn,  "[WARN]  ")
ARDUFLITE_SIM_LOG(error, "[ERROR] ")
ARDUFLITE_SIM_LOG(clear, "")

// ── ConfigRegistry ──────────────────────────────────────────────────────────
// No longer stubbed. The real registry compiles here now that it holds a
// hal::Mutex and std::string instead of a FreeRTOS handle and Arduino String
// (ADR-048), so host_sim exercises the actual configuration path — schema
// defaults, type checking and all — rather than a hand-written lookup that
// once returned 0.0f for outlimit and silently pinned both loops at zero.
