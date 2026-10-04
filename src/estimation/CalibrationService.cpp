/**
 * CalibrationService.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/estimation/CalibrationService.h"

namespace arduflite::estimation {

Status CalibrationService::request(Kind kind)
{
    // Refuse while a run is active OR while a finished one is unacknowledged.
    // Overwriting an unread Complete would silently discard offsets the caller
    // asked for and is about to store.
    const State current = _state.load(std::memory_order_acquire);
    if (current != State::Idle) { return Status::Busy; }

    if (_requested.load(std::memory_order_acquire)) { return Status::Busy; }

    // Kind first, then the flag: the sampling task reads the flag and only then
    // the kind, so publishing in this order means it can never see a request
    // paired with the previous run's kind.
    _requestedKind = kind;
    _requested.store(true, std::memory_order_release);
    return Status::Ok;
}

void CalibrationService::cancel()
{
    _requested.store(false, std::memory_order_release);
    _state.store(State::Idle, std::memory_order_release);
    _progressPercent.store(0, std::memory_order_relaxed);
}

void CalibrationService::acknowledge()
{
    const State current = _state.load(std::memory_order_acquire);
    if (current == State::Complete || current == State::Failed)
    {
        _state.store(State::Idle, std::memory_order_release);
        _progressPercent.store(0, std::memory_order_relaxed);
    }
}

void CalibrationService::finish(State outcome, Status error)
{
    _lastError = error;
    _progressPercent.store(outcome == State::Complete ? 100 : 0, std::memory_order_relaxed);

    // Published last, with release: a reader seeing Complete is guaranteed to
    // see the offsets and the error code that go with it.
    _state.store(outcome, std::memory_order_release);
}

void CalibrationService::service(const Vec3f& accel_g, const Vec3f& gyro_dps,
                                 float pressure_hpa, hal::Clock::time_point now)
{
    const State current = _state.load(std::memory_order_relaxed);

    // ── Start a pending run ─────────────────────────────────────────────────
    if (current == State::Idle && _requested.load(std::memory_order_acquire))
    {
        _kind = _requestedKind;
        _requested.store(false, std::memory_order_release);

        _start           = now;
        _sampleCount     = 0;
        _accelSum        = Vec3f{};
        _gyroSum         = Vec3f{};
        _pressureSum_hpa = 0.0f;

        _progressPercent.store(0, std::memory_order_relaxed);
        _state.store(State::Running, std::memory_order_release);

        // Deliberately no `return`: this tick's sample counts toward the run.
        // Skipping it would be harmless but makes the first tick a special case
        // in the tests for no reason.
    }

    if (_state.load(std::memory_order_relaxed) != State::Running) { return; }

    // ── Accumulate ──────────────────────────────────────────────────────────
    if (_kind == Kind::Barometric)
    {
        // A dead barometer reports nothing usable. Averaging in 0 hPa would drag
        // the ground reference down and offset every altitude for the flight, so
        // such ticks are skipped and simply do not count toward minimumSamples.
        if (pressure_hpa > 0.0f)
        {
            _pressureSum_hpa += pressure_hpa;
            ++_sampleCount;
        }
    }
    else
    {
        _accelSum.x += accel_g.x;
        _accelSum.y += accel_g.y;
        _accelSum.z += accel_g.z;
        _gyroSum.x  += gyro_dps.x;
        _gyroSum.y  += gyro_dps.y;
        _gyroSum.z  += gyro_dps.z;
        ++_sampleCount;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _start);

    // Progress is time-based, not sample-based: with a failing sensor the sample
    // count can stall while the run is still advancing toward its timeout, and a
    // progress bar frozen at 3% is less informative than one that reaches 100
    // and then reports failure.
    if (_config.duration.count() > 0)
    {
        const auto percent = (elapsed.count() * 100) / _config.duration.count();
        _progressPercent.store(static_cast<std::uint8_t>(percent > 100 ? 100 : percent),
                               std::memory_order_relaxed);
    }

    if (elapsed < _config.duration) { return; }

    // ── Finish ──────────────────────────────────────────────────────────────
    if (_sampleCount < _config.minimumSamples)
    {
        // A minimum, not just "more than zero": a run that managed a single
        // reading during a bus fault would otherwise be stored as gospel.
        finish(State::Failed, Status::IoError);
        return;
    }

    const float inverse = 1.0f / static_cast<float>(_sampleCount);

    if (_kind == Kind::Barometric)
    {
        _referencePressure_hpa = _pressureSum_hpa * inverse;
    }
    else
    {
        // Accelerometer: X and Y should read zero when level, Z should read +1 g.
        // Subtracting gravity from Z is what makes this an OFFSET rather than a
        // measurement — get it wrong and the aircraft believes it is in freefall
        // while sitting on the bench.
        _offsets.accel_g = Vec3f{ _accelSum.x * inverse,
                                  _accelSum.y * inverse,
                                  _accelSum.z * inverse - 1.0f };

        // Gyroscope: every axis should read zero at rest, so the average IS the bias.
        _offsets.gyro_dps = Vec3f{ _gyroSum.x * inverse,
                                   _gyroSum.y * inverse,
                                   _gyroSum.z * inverse };
    }

    finish(State::Complete, Status::Ok);
}

} // namespace arduflite::estimation
