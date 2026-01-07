#include "MotorPWM.h"
#include "esp_log.h"
#include <cmath>

MotorPWM::MotorPWM(gpio_num_t in1_pin, gpio_num_t in2_pin,
                   ledc_channel_t channel1, ledc_channel_t channel2,
                   ledc_timer_t timer, ledc_mode_t mode, uint32_t freq_hz,
                   uint8_t resolution_bits)
    : in1_pin_(in1_pin), in2_pin_(in2_pin), channel1_(channel1),
      channel2_(channel2), timer_(timer), mode_(mode), freq_hz_(freq_hz),
      resolution_bits_(resolution_bits) {}

void MotorPWM::begin() {
  configureTimer();
  configureChannel(channel1_, in1_pin_);
  configureChannel(channel2_, in2_pin_);

  // Stop motor initially
  ledc_set_duty(mode_, channel1_, 0);
  ledc_update_duty(mode_, channel1_);
  ledc_set_duty(mode_, channel2_, 0);
  ledc_update_duty(mode_, channel2_);
}

void MotorPWM::configureTimer() {
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode = mode_;
  timer_conf.duty_resolution = static_cast<ledc_timer_bit_t>(resolution_bits_);
  timer_conf.timer_num = timer_;
  timer_conf.freq_hz = freq_hz_;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;

  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
}

void MotorPWM::configureChannel(ledc_channel_t channel, gpio_num_t pin) {
  ledc_channel_config_t ch_conf = {};
  ch_conf.channel = channel;
  ch_conf.duty = 0;
  ch_conf.gpio_num = pin;
  ch_conf.speed_mode = mode_;
  ch_conf.hpoint = 0;
  ch_conf.timer_sel = timer_;
  ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
}

void MotorPWM::setDuty(float duty) {
  if (duty > 1.0f)
    duty = 1.0f;
  if (duty < -1.0f)
    duty = -1.0f;

  duty_ = duty;
  motor_stoped_ = duty == 0;
  direction_inverted_ = duty < 0;

  uint32_t max_duty = (1 << resolution_bits_) - 1;
  uint32_t scaled_duty = static_cast<uint32_t>(max_duty * fabs(duty));

  if (duty >= 0) {
    ledc_set_duty(mode_, channel1_, scaled_duty);
    ledc_update_duty(mode_, channel1_);
    ledc_set_duty(mode_, channel2_, 0);
    ledc_update_duty(mode_, channel2_);
  } else {
    ledc_set_duty(mode_, channel1_, 0);
    ledc_update_duty(mode_, channel1_);
    ledc_set_duty(mode_, channel2_, scaled_duty);
    ledc_update_duty(mode_, channel2_);
  }
}
