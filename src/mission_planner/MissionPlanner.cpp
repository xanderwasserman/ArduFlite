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
    if (xSemaphoreTake(_mutex, portMAX_DELAY)) 
    {
        _steps.clear();
        _steps.insert(_steps.end(), steps, steps + count);
        _running = false;
        xSemaphoreGive(_mutex);
    }
}

void MissionPlanner::start() 
{
    if (xSemaphoreTake(_mutex, portMAX_DELAY)) 
    {
        if (!_steps.empty() && !_running) 
        {
            _running = true;
            _currentIndex = 0;
            _stepStartMs = millis();
            auto &s = _steps[0];
            _ctrl.setDesiredEulerDegs(s.rollDeg, s.pitchDeg, s.yawDeg);
        }
        xSemaphoreGive(_mutex);
    }
}

void MissionPlanner::stop() 
{
    if (xSemaphoreTake(_mutex, portMAX_DELAY)) 
    {
        _running = false;
        xSemaphoreGive(_mutex);
    }
}

bool MissionPlanner::isRunning() 
{
    bool r = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY)) 
    {
        r = _running;
        xSemaphoreGive(_mutex);
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
        // only do work if mission is running
        if (isRunning()) 
        {
            if (xSemaphoreTake(_mutex, portMAX_DELAY)) 
            {
                uint32_t now = millis();
                if (_currentIndex < _steps.size()) 
                {
                    const auto &step = _steps[_currentIndex];
                    if (now - _stepStartMs >= step.holdMs) 
                    {
                        // advance
                        _currentIndex++;
                        if (_currentIndex >= _steps.size()) 
                        {
                            // finished
                            _running = false;
                        } 
                        else 
                        {
                            // set next step
                            _stepStartMs = now;
                            const auto &s = _steps[_currentIndex];
                            _ctrl.setDesiredEulerDegs(s.rollDeg, s.pitchDeg, s.yawDeg);
                        }
                    }
                }
                xSemaphoreGive(_mutex);
            }
        }
        // run at 100 Hz
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
