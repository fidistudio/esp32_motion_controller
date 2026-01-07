#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdbool.h>

// ------------------------------------------------
// MotorPWM
// Controls motor direction and speed using PWM
// ------------------------------------------------

class MotorPWM {
public:
  MotorPWM(gpio_num_t in1Pin, gpio_num_t in2Pin, ledc_channel_t forwardChannel,
           ledc_channel_t reverseChannel, ledc_timer_t timer = LEDC_TIMER_0,
           ledc_mode_t mode = LEDC_LOW_SPEED_MODE, uint32_t frequencyHz = 1000,
           uint8_t resolutionBits = 8);

  void begin();

  // Signed duty cycle: -1.0 ... +1.0
  void setDuty(float duty);

  float duty() const { return duty_; }
  bool isDirectionInverted() const { return isDirectionInverted_; }
  bool isStopped() const { return isStopped_; }

private:
  // ---- Setup helpers ----
  void configureTimer();
  void configureChannel(ledc_channel_t channel, gpio_num_t pin);

  // ---- Control helpers ----
  void applyForwardDuty(uint32_t duty);
  void applyReverseDuty(uint32_t duty);
  void stopMotor();

  uint32_t scaleDuty(float duty) const;

  // ---- Hardware configuration ----
  gpio_num_t in1Pin_;
  gpio_num_t in2Pin_;
  ledc_channel_t forwardChannel_;
  ledc_channel_t reverseChannel_;
  ledc_timer_t timer_;
  ledc_mode_t mode_;
  uint32_t frequencyHz_;
  uint8_t resolutionBits_;

  // ---- Runtime state ----
  float duty_ = 0.0f;
  bool isDirectionInverted_ = false;
  bool isStopped_ = true;
};
