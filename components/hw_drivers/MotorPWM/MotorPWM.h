#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdbool.h>

/* ================= PWM ================= */

class PWM
{
public:
  PWM(gpio_num_t pin, ledc_channel_t channel,
      ledc_timer_t timer, ledc_mode_t mode,
      ledc_timer_bit_t resolution);

  void setDuty(float duty);

private:
  void configureChannel(gpio_num_t pin, ledc_timer_t timer);

  ledc_channel_t channel_;
  ledc_mode_t mode_;
  uint32_t max_duty_;
};

/* ================= Motor ================= */

class Motor
{
public:
  Motor(gpio_num_t pin_a, gpio_num_t pin_b,
        ledc_channel_t ch_a, ledc_channel_t ch_b,
        ledc_timer_t timer, ledc_mode_t mode,
        ledc_timer_bit_t resolution);

  void stop();
  void setDuty(float new_duty); // -1.0 ... 1.0

private:
  PWM pwm_a_;
  PWM pwm_b_;
};