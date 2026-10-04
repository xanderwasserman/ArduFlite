/**
 * MissionPlanner.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 10 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/mission_planner/MissionPlanner.h"

#include <chrono>
#include <mutex>

#include "src/hal/board/Board.h"
#include "include/MissionConfiguration.h" 
#include "src/utils/Logging.h"

namespace {
/// Every public API takes this briefly; the loop takes it at 100 Hz.
constexpr std::chrono::milliseconds kMissionLockTimeout{ 5 };
} // namespace

MissionPlanner::MissionPlanner(ArduFliteController &controller)
    : _ctrl(controller)
    , _currentIndex(0)
    , _stepStartMs(0)
    , _running(false)
{
    // No platform resource here: this is constructed before Board::begin() has
    // run. The mutex is taken in begin() (ADR-030).
}

MissionPlanner::~MissionPlanner() 
{
    // Cooperative: asks the loop to leave at its next check.
    if (_task != nullptr) { _task->requestStop(); }
    _mutex = nullptr;   // pool-owned; nothing reclaims entries (ADR-011)
}

void MissionPlanner::begin() 
{
    auto& board = arduflite::board::Board::instance();

    auto mutex = board.allocMutex();
    if (!mutex)
    {
        LOG_ERR("MissionPlanner: failed to create mutex — planner disabled");
        return;
    }
    _mutex = mutex.value();

    // Load the default mission from mission_config.h. AFTER the mutex, because
    // loadMission() takes it and silently does nothing without one.
    loadMission(MissionConfig::MISSION_STEPS,
                sizeof(MissionConfig::MISSION_STEPS) / sizeof(MissionConfig::MISSION_STEPS[0]));

    arduflite::hal::TaskConfig config;
    config.name       = "MissionPlanner";
    config.stackBytes = 4096;
    config.priority   = arduflite::hal::Priority::Mission;

    auto task = board.scheduler().spawn(config, &taskEntry, this);
    if (!task)
    {
        LOG_ERR("MissionPlanner: failed to create task");
        _mutex = nullptr;
        return;
    }
    _task = task.value();
}

void MissionPlanner::loadMission(const Step *steps, size_t count) 
{
    {
        if (_mutex == nullptr) { return; }
        std::unique_lock lock(*_mutex, kMissionLockTimeout);
        if (!lock.owns_lock()) { return; }
        _steps.clear();
        _steps.insert(_steps.end(), steps, steps + count);
        _running = false;
    }
}

void MissionPlanner::start() 
{
    {
        if (_mutex == nullptr) { return; }
        std::unique_lock lock(*_mutex, kMissionLockTimeout);
        if (!lock.owns_lock()) { return; }
        if (!_steps.empty() && !_running) 
        {
            _running      = true;
            _currentIndex = 0;
            _stepStartMs = static_cast<std::uint32_t>(
                arduflite::board::Board::instance().clock().now()
                    .time_since_epoch().count() / 1000);

            // grab a const‐ref to the first step
            const auto &first   = _steps[_currentIndex];
            AttitudeDeg         stepSetpoint;
            stepSetpoint.roll   = first.rollDeg;
            stepSetpoint.pitch  = first.pitchDeg;
            stepSetpoint.yaw    = first.yawDeg;

            // apply it right away
            _ctrl.setAttitudeSetpoint(stepSetpoint);

            // log it
            LOG_INF("MissionPlanner: step %u → roll=%.1f°, pitch=%.1f°, yaw=%.1f°", (unsigned)_currentIndex, first.rollDeg, first.pitchDeg, first.yawDeg);
        }
    }
}

void MissionPlanner::stop() 
{
    {
        if (_mutex == nullptr) { return; }
        std::unique_lock lock(*_mutex, kMissionLockTimeout);
        if (!lock.owns_lock()) { return; }
        _running = false;
    }
}

bool MissionPlanner::isRunning() 
{
    bool r = false;

    {
        if (_mutex == nullptr) { return r; }
        std::unique_lock lock(*_mutex, kMissionLockTimeout);
        if (!lock.owns_lock()) { return r; }
        r = _running;
    }

    return r;
}

void MissionPlanner::taskEntry(void *pv) 
{
    static_cast<MissionPlanner*>(pv)->run();
}

void MissionPlanner::run() 
{
    auto&       board     = arduflite::board::Board::instance();
    auto&       scheduler = board.scheduler();
    const auto& clock     = board.clock();

    // `_task` is assigned only after spawn() RETURNS and the body may already be
    // running, so a null handle means "no stop possible yet" — not a crash.
    while (_task == nullptr || !_task->stopRequested())
    {
        {
            std::unique_lock lock(*_mutex, kMissionLockTimeout);
            if (!lock.owns_lock())
            {
                scheduler.sleepFor(std::chrono::milliseconds{ 10 });
                continue;
            }
            if (_running && !_steps.empty()) 
            {
                const std::uint32_t now = static_cast<std::uint32_t>(
                    clock.now().time_since_epoch().count() / 1000);
                const auto &cur = _steps[_currentIndex];

                // Have we held this step long enough?
                if (now - _stepStartMs >= cur.holdMs) 
                {
                    // advance index
                    _currentIndex++;

                    if (_currentIndex >= _steps.size()) 
                    {
                        // mission is done
                        LOG_INF("MissionPlanner: Mission complete");
                        _running = false;
                    } 
                    else 
                    {
                        // apply next step
                        const auto &next = _steps[_currentIndex];
                        AttitudeDeg         stepSetpoint;
                        stepSetpoint.roll   = next.rollDeg;
                        stepSetpoint.pitch  = next.pitchDeg;
                        stepSetpoint.yaw    = next.yawDeg;

                        _ctrl.setAttitudeSetpoint(stepSetpoint);
                        LOG_INF( "MissionPlanner: step %u → roll=%.1f°, pitch=%.1f°, yaw=%.1f°", (unsigned)_currentIndex, next.rollDeg, next.pitchDeg, next.yawDeg);
                        // reset the timer
                        _stepStartMs = now;
                    }
                }
            }
        }

        // throttle to 100 Hz
        scheduler.sleepFor(std::chrono::milliseconds{ 10 });
    }
}
