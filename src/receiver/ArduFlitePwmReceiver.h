/**
 * ArduFlitePwmReceiver.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDU_FLITE_PWM_RECEIVER_H
#define ARDU_FLITE_PWM_RECEIVER_H

#include <Arduino.h>

enum ReceiverChannelMode 
{
  UNIPOLAR, // Normalized output in [0, 1]
  BIPOLAR   // Normalized output in [-1, 1]
};

struct ReceiverChannelConfig 
{
  uint8_t pin;              // Digital pin for the PWM signal
  unsigned long minPulse;   // Minimum pulse width in microseconds
  unsigned long maxPulse;   // Maximum pulse width in microseconds
  ReceiverChannelMode mode; // Mode for normalization
};

class ArduFlitePwmReceiver 
{
public:
  /**
   * @brief Constructor for ArduFlitePwmReceiver.
   * @param configs Array of channel configurations.
   * @param numChannels Number of receiver channels.
   */
  ArduFlitePwmReceiver(const ReceiverChannelConfig* configs, size_t numChannels);

  /**
   * @brief Initializes the receiver by setting up the pins and attaching interrupts.
   */
  void begin();

  /**
   * @brief Gets the raw pulse width (in microseconds) for the specified channel.
   * @param channel Channel index.
   * @return Pulse width in microseconds.
   */
  unsigned long getPulseWidth(size_t channel);

  /**
   * @brief Gets the normalized value for a specified channel.
   *        UNIPOLAR channels return a value in [0, 1] while
   *        BIPOLAR channels return a value in [-1, 1].
   * @param channel Channel index.
   * @return Normalized value.
   */
  float getNormalizedValue(size_t channel);

private:
  size_t channelCount; // Number of channels
  ReceiverChannelConfig* channelConfigs; // Local copy of configurations
  
  // Arrays for storing the latest pulse widths and last rising edge times (in microseconds)
  volatile unsigned long* pulseWidths;
  volatile unsigned long* lastRiseTime;

  // Static pointer for ISR reference (assumes a single instance)
  static ArduFlitePwmReceiver* instance;

  /**
   * @brief Attaches an interrupt for a channel.
   * @param channel Channel index.
   */
  void attachChannelInterrupt(size_t channel);

  /**
   * @brief Static ISR handler for receiver channels.
   * @param arg Pointer to the channel index (passed as void*).
   */
  static void IRAM_ATTR handleInterrupt(void* arg);
};

#endif // ARDU_FLITE_PWM_RECEIVER_H
