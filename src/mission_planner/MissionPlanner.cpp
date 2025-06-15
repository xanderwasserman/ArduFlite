/**
 * MissionPlanner.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 10 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/mission_planner/MissionPlanner.h"
#include "include/MissionConfiguration.h" 
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

MissionPlanner::MissionPlanner(ArduFliteController &controller)
    : _ctrl(controller)
    , _currentIndex(0)
    , _stepStartMs(0)
    , _running(false)
    , _taskHandle(nullptr)
{
    // Create a mutex to protect all public APIs
    _mutex = xSemaphoreCreateMutex();
}

MissionPlanner::~MissionPlanner() 
{
    if (_taskHandle) 
    {
        vTaskDelete(_taskHandle);
    }
    if (_mutex) 
    {
        vSemaphoreDelete(_mutex);
    }
}

void MissionPlanner::begin() 
{
    // Load the default mission from mission_config.h
    loadMission(MissionConfig::MISSION_STEPS, sizeof(MissionConfig::MISSION_STEPS)/sizeof(MissionConfig::MISSION_STEPS[0]));

    // Launch the RTOS task
    xTaskCreate(
        taskEntry, 
        "MissionPlanner", 
        4096, 
        this,
        tskIDLE_PRIORITY + 1, 
        &_taskHandle
    );
}

void MissionPlanner::loadMission(const Step *steps, size_t count) 
{
    {
        SemaphoreLock lock(_mutex);
        _steps.clear();
        _steps.insert(_steps.end(), steps, steps + count);
        _running = false;
    }
}

void MissionPlanner::start() 
{
    {
        SemaphoreLock lock(_mutex);
        if (!_steps.empty() && !_running) 
        {
            _running      = true;
            _currentIndex = 0;
            _stepStartMs  = millis();

            // grab a const‐ref to the first step
            const auto &first   = _steps[_currentIndex];
            EulerAngles         stepSetpoint;
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
        SemaphoreLock lock(_mutex);
        _running = false;
    }
}

bool MissionPlanner::isRunning() 
{
    bool r = false;

    {
        SemaphoreLock lock(_mutex);
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
    while (true) 
    {
        {
            SemaphoreLock lock(_mutex);
            if (_running && !_steps.empty()) 
            {
                uint32_t now = millis();
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
                        EulerAngles         stepSetpoint;
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

