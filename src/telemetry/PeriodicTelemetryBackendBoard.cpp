/**
 * PeriodicTelemetryBackendBoard.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief The one part of the backend lifecycle that needs a concrete board.
 *
 * Separate translation unit so the rest of PeriodicTelemetryBackend stays free
 * of Board — and therefore host-testable. Tests drive beginWith() with a
 * HostMutex and a RecordingScheduler; this supplies the board's.
 */
#include "src/telemetry/PeriodicTelemetryBackend.h"

#include "src/hal/board/Board.h"
#include "src/utils/Logging.h"

void PeriodicTelemetryBackend::begin()
{
    auto& board = arduflite::board::Board::instance();

    auto mutex = board.allocMutex();
    if (!mutex)
    {
        LOG_ERR("%s: no mutex available — backend disabled", taskName());
        return;
    }

    (void)beginWith(mutex.value(), board.scheduler());
}
