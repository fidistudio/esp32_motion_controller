#include "Encoder.h"
#include "esp_log.h"
#include <string.h>

portMUX_TYPE Encoder::encoder_mux = portMUX_INITIALIZER_UNLOCKED;

Encoder::Encoder(gpio_num_t encoder_gpio, gpio_num_t sector_gpio)
    : encoder_gpio_(encoder_gpio), sector_gpio_(sector_gpio),
      encoder_unit_(nullptr), sector_unit_(nullptr) {
  memset(&state_, 0, sizeof(state_));
}

void Encoder::begin() {
  setupEncoderPCNT();
  setupSectorPCNT();
}

// ---------------- PCNT Helpers ----------------

void Encoder::configurePCNTUnit(pcnt_unit_handle_t *unit) {
  pcnt_unit_config_t config;
  memset(&config, 0, sizeof(config));
  config.high_limit = PCNT_HIGH_LIMIT;
  config.low_limit = PCNT_LOW_LIMIT;

  ESP_ERROR_CHECK(pcnt_new_unit(&config, unit));
}

void Encoder::configurePCNTChannel(pcnt_unit_handle_t unit, gpio_num_t gpio) {
  pcnt_channel_handle_t channel;
  pcnt_chan_config_t config;
  memset(&config, 0, sizeof(config));
  config.edge_gpio_num = gpio;
  config.level_gpio_num = -1;

  ESP_ERROR_CHECK(pcnt_new_channel(unit, &config, &channel));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
}

void Encoder::configureGlitchFilter(pcnt_unit_handle_t unit) {
  pcnt_glitch_filter_config_t filter;
  memset(&filter, 0, sizeof(filter));
  filter.max_glitch_ns = GLITCH_FILTER_NS;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));
}

void Encoder::startPCNT(pcnt_unit_handle_t unit) {
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

// ---------------- Encoder / Sector Setup ----------------

void Encoder::setupEncoderPCNT() {
  configurePCNTUnit(&encoder_unit_);
  configurePCNTChannel(encoder_unit_, encoder_gpio_);
  configureGlitchFilter(encoder_unit_);

  pcnt_event_callbacks_t callbacks;
  memset(&callbacks, 0, sizeof(callbacks));
  callbacks.on_reach = onEncoderPulse;
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(encoder_unit_, &callbacks, &state_));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit_, 1));

  startPCNT(encoder_unit_);
}

void Encoder::setupSectorPCNT() {
  configurePCNTUnit(&sector_unit_);
  configurePCNTChannel(sector_unit_, sector_gpio_);
  configureGlitchFilter(sector_unit_);

  pcnt_event_callbacks_t callbacks;
  memset(&callbacks, 0, sizeof(callbacks));
  callbacks.on_reach = onSectorReset;
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(sector_unit_, &callbacks, &state_));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(sector_unit_, 1));

  startPCNT(sector_unit_);
}

// ---------------- ISR Handlers ----------------

bool IRAM_ATTR Encoder::onEncoderPulse(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t *edata,
                                       void *user_ctx) {
  encoder_state_t *state = reinterpret_cast<encoder_state_t *>(user_ctx);
  int64_t now = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&encoder_mux);
  if (state->last_pulse_time_us != 0) {
    state->pulse_interval_us = now - state->last_pulse_time_us;

    if (state->calibrating && state->revolutions < MAX_CALIBRATION_REVS) {
      state->sector_time[state->current_sector][state->revolutions] =
          state->pulse_interval_us;
    }
  }

  state->last_pulse_time_us = now;
  state->current_sector = (state->current_sector + 1) % PULSES_PER_REV;
  portEXIT_CRITICAL_ISR(&encoder_mux);

  pcnt_unit_clear_count(unit);
  return false;
}

bool IRAM_ATTR Encoder::onSectorReset(pcnt_unit_handle_t unit,
                                      const pcnt_watch_event_data_t *edata,
                                      void *user_ctx) {
  encoder_state_t *state = reinterpret_cast<encoder_state_t *>(user_ctx);

  portENTER_CRITICAL_ISR(&encoder_mux);
  state->current_sector = 0;

  if (state->calibration_requested && !state->calibrating) {
    state->calibrating = true;
    state->calibration_requested = false;
    state->revolutions = 0;
    memset(state->sector_time, 0, sizeof(state->sector_time));
  } else if (state->calibrating) {
    state->revolutions++;
  }
  portEXIT_CRITICAL_ISR(&encoder_mux);

  pcnt_unit_clear_count(unit);
  return false;
}

// ---------------- Compute Speed ----------------

float Encoder::computeRadPerSec() {
  int64_t interval_us;
  int sector;
  float correction = 1.0f;

  taskENTER_CRITICAL(&encoder_mux);
  interval_us = state_.pulse_interval_us;
  sector = (state_.current_sector - 1 + PULSES_PER_REV) % PULSES_PER_REV;
  taskEXIT_CRITICAL(&encoder_mux);

  if (interval_us <= 0)
    return 0.0f;

  if (state_.use_lut_correction) {
    int dir_idx = state_.direction_inverted ? 1 : 0;
    correction =
        1.0f / (state_.sector_correction_factor[sector][dir_idx] / 100.0f);
  }

  const float T = interval_us * correction * 1e-6f;
  return (2.0f * static_cast<float>(M_PI)) / (PULSES_PER_REV * T);
}

float Encoder::computeRPM() {
  int64_t interval_us;
  int sector;
  float correction = 1.0f;

  taskENTER_CRITICAL(&encoder_mux);
  interval_us = state_.pulse_interval_us;
  sector = (state_.current_sector - 1 + PULSES_PER_REV) % PULSES_PER_REV;
  taskEXIT_CRITICAL(&encoder_mux);

  if (interval_us <= 0)
    return 0.0f;

  if (state_.use_lut_correction) {
    int dir_idx = state_.direction_inverted ? 1 : 0;
    correction =
        1.0f / (state_.sector_correction_factor[sector][dir_idx] / 100.0f);
  }

  const float T = interval_us * correction * 1e-6f;
  return 60.0f / (PULSES_PER_REV * T);
}
