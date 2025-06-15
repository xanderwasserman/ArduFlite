/**
 * ArdufliteCRSFCallbacks.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_CSRF_CALLBACKS_H
#define ARDUFLITE_CSRF_CALLBACKS_H

#include "src/controller/ArduFliteController.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/ControlMixer.h"

#include <Arduino.h>

namespace CRSFCallbacks {
  void onFailsafe();
  void onRoll(uint8_t ch, float v);
  void onPitch(uint8_t ch, float v);
  void onYaw(uint8_t ch, float v);
  void onModeSwitch(uint8_t ch, float v);
  void onActivateMission(uint8_t ch, float v);
  void onArm(uint8_t ch, float v);
}

#endif //ARDUFLITE_CSRF_CALLBACKS_H