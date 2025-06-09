/**
 * ControlMixer.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 09 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ControlMixer.h"

ControlMixer::ControlMixer()
  : currentMode_(ASSIST_MODE)
  , raw_{0.0f,0.0f,0.0f}
  , maxAttDeg_(45.0f)
  , maxRateDeg_(180.0f)
#ifdef ENABLE_SAFE_MIXING
  , mixYtoR_(0.5f)
  , mixPtoR_(0.5f)
#endif
#ifdef ENABLE_SINGLE_AXIS_CONTROL
  , axisEn_{ true, true, true }
#endif
{}

void ControlMixer::begin(float maxAtt, float maxRate) {
    maxAttDeg_  = maxAtt;
    maxRateDeg_ = maxRate;
}

void ControlMixer::setMode(ArduFliteMode m) {
    currentMode_ = m;
}

void ControlMixer::setRawRoll(float v) {
    raw_[0] = constrain(v, -1.0f, 1.0f);
    computeAndPush_();
}

void ControlMixer::setRawPitch(float v) {
    raw_[1] = constrain(v, -1.0f, 1.0f);
    computeAndPush_();
}

void ControlMixer::setRawYaw(float v) {
    raw_[2] = constrain(v, -1.0f, 1.0f);
    computeAndPush_();
}

#ifdef ENABLE_SAFE_MIXING
