/**
 * MissionPlanner.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 10 May 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef MISSION_PLANNER_H
#define MISSION_PLANNER_H

#include <Arduino.h>
#include <vector>
#include "src/controller/ArduFliteController.h"

/**
 * Simple sequencer that steps through a list of attitude set-points.
 * Runs in its own FreeRTOS task.
 */
class MissionPlanner {
public:
    struct Step {
        float   rollDeg;   // desired roll angle in degrees
        float   pitchDeg;  // desired pitch angle
        float   yawDeg;    // desired yaw angle
        uint32_t holdMs;   // how long to hold this step (ms)
    };

    explicit MissionPlanner(ArduFliteController &controller);
    ~MissionPlanner();

    /// Must be called once (e.g. in setup()) to create the RTOS task.
    void begin();

    /// Replace the current mission with a new sequence of steps.
    void loadMission(const Step *steps, size_t count);

    /// Start executing from the first step (only when in-flight).
    void start();

    /// Immediately abort the mission.
    void stop();

    /// Query whether the mission is currently running.
    bool isRunning();

private:
    static void  taskEntry(void *pv);
    void         run();

    ArduFliteController & _ctrl;
    std::vector<Step>     _steps;
    size_t                _currentIndex;
    uint32_t              _stepStartMs;
    bool                  _running;
    TaskHandle_t          _taskHandle;
    SemaphoreHandle_t     _mutex;
};


#endif // MISSION_PLANNER_H