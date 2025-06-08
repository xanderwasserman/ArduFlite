/**
 * ArduFlitePwmReceiver.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
 #include "src/receiver/pwm/ArduFlitePwmReceiver.h"

// Initialize static instance pointer to nullptr.
ArduFlitePwmReceiver* ArduFlitePwmReceiver::instance = nullptr;

ArduFlitePwmReceiver::ArduFlitePwmReceiver(const ReceiverChannelConfig* configs, size_t numChannels)
  : channelCount(numChannels)
{
  // Allocate and copy channel configurations.
  channelConfigs = new ReceiverChannelConfig[numChannels];
  for (size_t i = 0; i < numChannels; i++) {
    channelConfigs[i] = configs[i];
  }
  
  // Allocate arrays for pulse widths and last rise times.
  pulseWidths = new volatile unsigned long[numChannels];
  lastRiseTime = new volatile unsigned long[numChannels];
  for (size_t i = 0; i < numChannels; i++) {
    pulseWidths[i] = 0;
    lastRiseTime[i] = 0;
  }
  
  // Set the static instance pointer.
  instance = this;
}

void ArduFlitePwmReceiver::begin() {
  // Configure each channel's pin as input and attach its interrupt.
  for (size_t i = 0; i < channelCount; i++) {
    pinMode(channelConfigs[i].pin, INPUT);
    attachChannelInterrupt(i);
  }
}

void ArduFlitePwmReceiver::attachChannelInterrupt(size_t channel) {
  // ESP32 supports attachInterruptArg so we can pass the channel as an argument.
  attachInterruptArg(digitalPinToInterrupt(channelConfigs[channel].pin), handleInterrupt, (void*) channel, CHANGE);
}

void IRAM_ATTR ArduFlitePwmReceiver::handleInterrupt(void* arg) {
  size_t channel = (size_t)arg;
  if (instance == nullptr) return;
  
  int state = digitalRead(instance->channelConfigs[channel].pin);
  unsigned long currentTime = micros();
  
  if (state == HIGH) {
    // On rising edge, capture the timestamp.
    instance->lastRiseTime[channel] = currentTime;
  } else {
    // On falling edge, calculate pulse width.
    unsigned long start = instance->lastRiseTime[channel];
    instance->pulseWidths[channel] = (currentTime >= start) ? currentTime - start : 0;
  }
}

unsigned long ArduFlitePwmReceiver::getPulseWidth(size_t channel) {
  if (channel < channelCount) {
    noInterrupts();
    unsigned long value = pulseWidths[channel];
    interrupts();
    return value;
  }
  return 0;
}

float ArduFlitePwmReceiver::getNormalizedValue(size_t channel) {
  if (channel >= channelCount) return 0.0f;
  unsigned long pw = getPulseWidth(channel);
  const ReceiverChannelConfig &cfg = channelConfigs[channel];
  
  if (cfg.mode == UNIPOLAR) {
    if (pw <= cfg.minPulse) return 0.0f;
    if (pw >= cfg.maxPulse) return 1.0f;
    return (float)(pw - cfg.minPulse) / (float)(cfg.maxPulse - cfg.minPulse);
  } else { // Bipolar
    unsigned long neutral = (cfg.minPulse + cfg.maxPulse) / 2;
    if (pw <= neutral) {
      if (neutral == cfg.minPulse) return -1.0f;
      return -1.0f + (float)(pw - cfg.minPulse) / (float)(neutral - cfg.minPulse);
    } else {
      if (cfg.maxPulse == neutral) return 1.0f;
      return (float)(pw - neutral) / (float)(cfg.maxPulse - neutral);
    }
  }
}
