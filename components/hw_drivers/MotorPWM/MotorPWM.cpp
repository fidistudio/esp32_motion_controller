#include "MotorPWM.h"
#include "esp_log.h"
#include <cmath>

/* ================= Motor ================= */

Motor::Motor(gpio_num_t pin_a, gpio_num_t pin_b, ledc_channel_t ch_a,
             ledc_channel_t ch_b, ledc_timer_t timer, ledc_mode_t mode,
             ledc_timer_bit_t resolution)
    : pwm_a_(pin_a, ch_a, timer, mode, resolution),
      pwm_b_(pin_b, ch_b, timer, mode, resolution) {
  stop();
}

void Motor::stop() {
  pwm_a_.setDuty(0.0f);
  pwm_b_.setDuty(0.0f);
}

void Motor::setDuty(float new_duty) {
  if (new_duty > 0.0f) {
    pwm_a_.setDuty(new_duty);
    pwm_b_.setDuty(0.0f);
  } else if (new_duty < 0.0f) {
    pwm_a_.setDuty(0.0f);
    pwm_b_.setDuty(-new_duty);
  } else {
    stop();
  }
}

/* ================= PWM ================= */

PWM::PWM(gpio_num_t pin, ledc_channel_t channel, ledc_timer_t timer,
         ledc_mode_t mode, ledc_timer_bit_t resolution)
    : channel_(channel), mode_(mode) {
  max_duty_ = (1U << resolution) - 1;
  configureChannel(pin, timer);
}

void PWM::setDuty(float new_duty) {
  if (new_duty < 0.0f)
    new_duty = 0.0f;
  if (new_duty > 1.0f)
    new_duty = 1.0f;

  float mapped_duty;

  const float min_duty = 0.15f;

  if (new_duty == 0.0f) {
    mapped_duty = 0.0f; // motor detenido
  } else {
    mapped_duty = min_duty + (1.0f - min_duty) * new_duty;
  }

  ledc_set_duty(mode_, channel_,
                static_cast<uint32_t>(max_duty_ * mapped_duty));
  ledc_update_duty(mode_, channel_);
}

void PWM::configureChannel(gpio_num_t pin, ledc_timer_t timer) {
  ledc_channel_config_t ch_conf = {};
  ch_conf.channel = channel_;
  ch_conf.duty = 0;
  ch_conf.gpio_num = pin;
  ch_conf.speed_mode = mode_;
  ch_conf.hpoint = 0;
  ch_conf.timer_sel = timer;
  ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
}
