#include "MotorPWM.h"
#include <cmath>

// ---------------- Constructor ----------------

MotorPWM::MotorPWM(gpio_num_t in1Pin, gpio_num_t in2Pin,
                   ledc_channel_t forwardChannel, ledc_channel_t reverseChannel,
                   ledc_timer_t timer, ledc_mode_t mode, uint32_t frequencyHz,
                   uint8_t resolutionBits)
    : in1Pin_(in1Pin), in2Pin_(in2Pin), forwardChannel_(forwardChannel),
      reverseChannel_(reverseChannel), timer_(timer), mode_(mode),
      frequencyHz_(frequencyHz), resolutionBits_(resolutionBits) {}

// ---------------- Initialization ----------------

void MotorPWM::begin() {
  configureTimer();
  configureChannel(forwardChannel_, in1Pin_);
  configureChannel(reverseChannel_, in2Pin_);

  stopMotor();
}

// ---------------- Public Control ----------------

void MotorPWM::setDuty(float duty) {
  if (duty > 1.0f)
    duty = 1.0f;
  if (duty < -1.0f)
    duty = -1.0f;

  duty_ = duty;
  isStopped_ = (duty == 0.0f);
  isDirectionInverted_ = (duty < 0.0f);

  if (isStopped_) {
    stopMotor();
    return;
  }

  const uint32_t scaledDuty = scaleDuty(duty);

  if (duty > 0.0f) {
    applyForwardDuty(scaledDuty);
  } else {
    applyReverseDuty(scaledDuty);
  }
}

// ---------------- Setup Helpers ----------------

void MotorPWM::configureTimer() {
  ledc_timer_config_t timerConf{};
  timerConf.speed_mode = mode_;
  timerConf.duty_resolution = static_cast<ledc_timer_bit_t>(resolutionBits_);
  timerConf.timer_num = timer_;
  timerConf.freq_hz = frequencyHz_;
  timerConf.clk_cfg = LEDC_AUTO_CLK;

  ESP_ERROR_CHECK(ledc_timer_config(&timerConf));
}

void MotorPWM::configureChannel(ledc_channel_t channel, gpio_num_t pin) {
  ledc_channel_config_t channelConf{};
  channelConf.channel = channel;
  channelConf.duty = 0;
  channelConf.gpio_num = pin;
  channelConf.speed_mode = mode_;
  channelConf.hpoint = 0;
  channelConf.timer_sel = timer_;

  ESP_ERROR_CHECK(ledc_channel_config(&channelConf));
}

// ---------------- Control Helpers ----------------

void MotorPWM::applyForwardDuty(uint32_t duty) {
  ledc_set_duty(mode_, forwardChannel_, duty);
  ledc_update_duty(mode_, forwardChannel_);

  ledc_set_duty(mode_, reverseChannel_, 0);
  ledc_update_duty(mode_, reverseChannel_);
}

void MotorPWM::applyReverseDuty(uint32_t duty) {
  ledc_set_duty(mode_, forwardChannel_, 0);
  ledc_update_duty(mode_, forwardChannel_);

  ledc_set_duty(mode_, reverseChannel_, duty);
  ledc_update_duty(mode_, reverseChannel_);
}

void MotorPWM::stopMotor() {
  ledc_set_duty(mode_, forwardChannel_, 0);
  ledc_update_duty(mode_, forwardChannel_);

  ledc_set_duty(mode_, reverseChannel_, 0);
  ledc_update_duty(mode_, reverseChannel_);
}

// ---------------- Math Helpers ----------------

uint32_t MotorPWM::scaleDuty(float duty) const {
  const uint32_t maxDuty = (1U << resolutionBits_) - 1;
  return static_cast<uint32_t>(maxDuty * std::fabs(duty));
}
