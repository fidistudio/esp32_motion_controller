#pragma once

#include "esp_err.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
}

#define PULSES_PER_REV 15
#define MAX_CALIBRATION_REVS 15
#define GLITCH_FILTER_NS 10000 // 10 µs
#define PCNT_HIGH_LIMIT 2
#define PCNT_LOW_LIMIT -1

typedef struct {
  volatile int64_t last_pulse_time_us;
  volatile int64_t pulse_interval_us;
  volatile int current_sector;

  bool direction_inverted;
  bool use_lut_correction;

  bool calibration_requested;
  bool calibrating;
  bool velocity_reseted;
  int revolutions;

  int64_t sector_time[PULSES_PER_REV][MAX_CALIBRATION_REVS];
  float sector_correction_factor[PULSES_PER_REV][2];
} encoder_state_t;

class Encoder {
public:
  encoder_state_t state_;

  Encoder(gpio_num_t encoder_gpio, gpio_num_t sector_gpio);

  void begin();
  bool isCalibrating() const { return state_.calibrating; }
  void requestCalibration() { state_.calibration_requested = true; }
  void setDirectionInverted(bool inverted) {
    if (state_.direction_inverted != inverted) {
      int current = state_.current_sector;
      state_.current_sector = (PULSES_PER_REV - current) % PULSES_PER_REV;
    }
    state_.direction_inverted = inverted;
  }

  void isVelocityReseted(bool isMotorOff) {
    state_.velocity_reseted = isMotorOff;

    if (isMotorOff) {
      state_.pulse_interval_us = 0;
      state_.last_pulse_time_us = 0;

      if (encoder_unit_) {
        pcnt_unit_stop(encoder_unit_);
        pcnt_unit_clear_count(encoder_unit_);
        pcnt_unit_start(encoder_unit_);
      }
    }
  }

  float computeRadPerSec();
  float computeRPM();

private:
  gpio_num_t encoder_gpio_;
  gpio_num_t sector_gpio_;
  pcnt_unit_handle_t encoder_unit_;
  pcnt_unit_handle_t sector_unit_;

  void configurePCNTUnit(pcnt_unit_handle_t *unit);
  void configurePCNTChannel(pcnt_unit_handle_t unit, gpio_num_t gpio);
  void configureGlitchFilter(pcnt_unit_handle_t unit);
  void startPCNT(pcnt_unit_handle_t unit);
  void setupEncoderPCNT();
  void setupSectorPCNT();

  static bool IRAM_ATTR onEncoderPulse(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t *edata,
                                       void *user_ctx);
  static bool IRAM_ATTR onSectorReset(pcnt_unit_handle_t unit,
                                      const pcnt_watch_event_data_t *edata,
                                      void *user_ctx);

  static portMUX_TYPE encoder_mux; // mutex for ISR and tasks
};
