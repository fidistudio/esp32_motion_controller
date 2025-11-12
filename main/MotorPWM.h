#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdbool.h>

class MotorPWM {
public:
  MotorPWM(gpio_num_t in1_pin, gpio_num_t in2_pin, ledc_channel_t channel1,
           ledc_channel_t channel2, ledc_timer_t timer = LEDC_TIMER_0,
           ledc_mode_t mode = LEDC_LOW_SPEED_MODE, uint32_t freq_hz = 1000,
           uint8_t resolution_bits = 8);

  void begin();
  void setDuty(float duty); // -1.0 ... 1.0
  bool isDirectionInverted() const { return direction_inverted_; }

private:
  void configureTimer();
  void configureChannel(ledc_channel_t channel, gpio_num_t pin);

  gpio_num_t in1_pin_;
  gpio_num_t in2_pin_;
  ledc_channel_t channel1_;
  ledc_channel_t channel2_;
  ledc_timer_t timer_;
  ledc_mode_t mode_;
  uint32_t freq_hz_;
  uint8_t resolution_bits_;

  bool direction_inverted_ = false;
};
