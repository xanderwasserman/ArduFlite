/**
 * test_motion_signals.cpp - Host tests for IMU launch/stable motion logic.
 *
 * Mirrors the threshold/debounce arithmetic used by ArduFliteIMU::updateMotionSignals()
 * so boundary behavior is locked down without requiring Arduino sensor hardware.
 */
#include <gtest/gtest.h>

class MotionDetectorHarness {
public:
    void feed(float accelMagSq, float gyroSq, unsigned long nowMs)
    {
        constexpr float ACC_THROW_THR      = 0.10f;
        constexpr float ACC_STABLE_THR     = 0.30f;
        constexpr float GYRO_THROW_MIN     = 15.0f;
        constexpr float GYRO_THROW_MAX     = 150.0f;
        constexpr float GYRO_STABLE_THR    = 2.0f;
        constexpr float GYRO_THROW_MIN_SQ  = GYRO_THROW_MIN  * GYRO_THROW_MIN;
        constexpr float GYRO_THROW_MAX_SQ  = GYRO_THROW_MAX  * GYRO_THROW_MAX;
        constexpr float GYRO_STABLE_THR_SQ = GYRO_STABLE_THR * GYRO_STABLE_THR;
        constexpr float ACC_THROW_HI_SQ    = (1.0f + ACC_THROW_THR)  * (1.0f + ACC_THROW_THR);
        constexpr float ACC_THROW_LO_SQ    = (1.0f - ACC_THROW_THR)  * (1.0f - ACC_THROW_THR);
        constexpr float ACC_STABLE_HI_SQ   = (1.0f + ACC_STABLE_THR) * (1.0f + ACC_STABLE_THR);
        constexpr float ACC_STABLE_LO_SQ   = (1.0f - ACC_STABLE_THR) * (1.0f - ACC_STABLE_THR);
        constexpr unsigned long DEBOUNCE   = 50;
        constexpr unsigned long STABLE_MS  = 2000;

        const bool accelThrow = (accelMagSq > ACC_THROW_HI_SQ || accelMagSq < ACC_THROW_LO_SQ);
        if ((accelThrow || gyroSq > GYRO_THROW_MIN_SQ) && gyroSq < GYRO_THROW_MAX_SQ)
        {
            launchDetected = (nowMs - motionStartTime > DEBOUNCE);
        }
        else
        {
            motionStartTime = nowMs;
            launchDetected = false;
        }

        const bool accelStable = (accelMagSq >= ACC_STABLE_LO_SQ && accelMagSq <= ACC_STABLE_HI_SQ);
        if (accelStable && gyroSq < GYRO_STABLE_THR_SQ)
        {
            stableDetected = (nowMs - flightStableStartTime >= STABLE_MS);
        }
        else
        {
            flightStableStartTime = nowMs;
            stableDetected = false;
        }
    }

    bool launchDetected = false;
    bool stableDetected = false;

private:
    unsigned long motionStartTime = 0;
    unsigned long flightStableStartTime = 0;
};

TEST(MotionSignals, GyroOnlyLaunchRequiresDebounce)
{
    MotionDetectorHarness h;

    h.feed(1.0f, 16.0f * 16.0f, 0);
    EXPECT_FALSE(h.launchDetected);

    h.feed(1.0f, 16.0f * 16.0f, 50);
    EXPECT_FALSE(h.launchDetected);

    h.feed(1.0f, 16.0f * 16.0f, 51);
    EXPECT_TRUE(h.launchDetected);
}

TEST(MotionSignals, GyroThrowMaxRejectsLaunch)
{
    MotionDetectorHarness h;

    h.feed(1.0f, 151.0f * 151.0f, 0);
    h.feed(1.0f, 151.0f * 151.0f, 100);

    EXPECT_FALSE(h.launchDetected);
}

TEST(MotionSignals, AccelThrowThresholdsAreStrict)
{
    MotionDetectorHarness h;
    constexpr float ACC_THROW_HI_SQ = 1.1f * 1.1f;
    constexpr float ACC_THROW_LO_SQ = 0.9f * 0.9f;

    h.feed(ACC_THROW_HI_SQ, 0.0f, 0);
    h.feed(ACC_THROW_HI_SQ, 0.0f, 100);
    EXPECT_FALSE(h.launchDetected);

    h.feed(ACC_THROW_HI_SQ + 0.001f, 0.0f, 101);
    h.feed(ACC_THROW_HI_SQ + 0.001f, 0.0f, 152);
    EXPECT_TRUE(h.launchDetected);

    h.feed(ACC_THROW_LO_SQ, 0.0f, 200);
    h.feed(ACC_THROW_LO_SQ, 0.0f, 300);
    EXPECT_FALSE(h.launchDetected);
}

TEST(MotionSignals, StableRequiresInclusiveAccelBoundsAndExclusiveGyroBound)
{
    MotionDetectorHarness h;
    constexpr float ACC_STABLE_HI_SQ = 1.3f * 1.3f;
    constexpr float ACC_STABLE_LO_SQ = 0.7f * 0.7f;

    h.feed(ACC_STABLE_HI_SQ, 1.0f, 0);
    h.feed(ACC_STABLE_HI_SQ, 1.0f, 1999);
    EXPECT_FALSE(h.stableDetected);

    h.feed(ACC_STABLE_HI_SQ, 1.0f, 2000);
    EXPECT_TRUE(h.stableDetected);

    h.feed(ACC_STABLE_LO_SQ, 2.0f * 2.0f, 3000);
    h.feed(ACC_STABLE_LO_SQ, 2.0f * 2.0f, 6000);
    EXPECT_FALSE(h.stableDetected);
}
