#include "Encoder.h"
#include "esp_log.h"
#include <string.h>

portMUX_TYPE Encoder::encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- Constructor ----------------

Encoder::Encoder(gpio_num_t encoder_gpio, gpio_num_t sector_gpio)
    : encoder_gpio_(encoder_gpio), sector_gpio_(sector_gpio),
      encoder_unit_(nullptr), sector_unit_(nullptr) {
  memset(&state_, 0, sizeof(state_));
}

// ---------------- Public API ----------------

void Encoder::begin() {
  setupEncoderPCNT();
  setupSectorPCNT();
}

void Encoder::invertDirection() {
  taskENTER_CRITICAL(&encoder_mux);
  state_.isDirectionInverted = true;
  remapCurrentSector();
  taskEXIT_CRITICAL(&encoder_mux);
}

void Encoder::setDirectionNormal() {
  taskENTER_CRITICAL(&encoder_mux);
  state_.isDirectionInverted = false;
  remapCurrentSector();
  taskEXIT_CRITICAL(&encoder_mux);
}

void Encoder::resetVelocity() {
  taskENTER_CRITICAL(&encoder_mux);
  state_.velocityReset = true;
  state_.pulseIntervalUs = 0;
  state_.lastPulseTimeUs = 0;
  taskEXIT_CRITICAL(&encoder_mux);

  if (encoder_unit_) {
    pcnt_unit_stop(encoder_unit_);
    pcnt_unit_clear_count(encoder_unit_);
    pcnt_unit_start(encoder_unit_);
  }
}

void Encoder::enableVelocityTracking() {
  taskENTER_CRITICAL(&encoder_mux);
  state_.velocityReset = false;
  taskEXIT_CRITICAL(&encoder_mux);
}

// ---------------- PCNT Helpers ----------------

void Encoder::configurePCNTUnit(pcnt_unit_handle_t *unit) {
  pcnt_unit_config_t config{};
  config.high_limit = PCNT_SINGLE_PULSE_HIGH;
  config.low_limit = PCNT_SINGLE_PULSE_LOW;

  ESP_ERROR_CHECK(pcnt_new_unit(&config, unit));
}

void Encoder::configurePCNTChannel(pcnt_unit_handle_t unit, gpio_num_t gpio) {
  pcnt_channel_handle_t channel;
  pcnt_chan_config_t config{};
  config.edge_gpio_num = gpio;
  config.level_gpio_num = -1;

  ESP_ERROR_CHECK(pcnt_new_channel(unit, &config, &channel));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
}

void Encoder::configureGlitchFilter(pcnt_unit_handle_t unit) {
  pcnt_glitch_filter_config_t filter{};
  filter.max_glitch_ns = GLITCH_FILTER_NS;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));
}

void Encoder::startPCNT(pcnt_unit_handle_t unit) {
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

// ---------------- Setup ----------------

void Encoder::setupEncoderPCNT() {
  configurePCNTUnit(&encoder_unit_);
  configurePCNTChannel(encoder_unit_, encoder_gpio_);
  configureGlitchFilter(encoder_unit_);

  pcnt_event_callbacks_t callbacks{};
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

  pcnt_event_callbacks_t callbacks{};
  callbacks.on_reach = onSectorReset;

  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(sector_unit_, &callbacks, &state_));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(sector_unit_, 1));

  startPCNT(sector_unit_);
}

// ---------------- ISR Callbacks ----------------

bool IRAM_ATTR Encoder::onEncoderPulse(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t *,
                                       void *user_ctx) {

  auto *state = static_cast<EncoderRuntimeState *>(user_ctx);
  const int64_t now = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&encoder_mux);
  updatePulseTiming(state, now);
  advanceSector(state);
  portEXIT_CRITICAL_ISR(&encoder_mux);

  pcnt_unit_clear_count(unit);
  return false;
}

bool IRAM_ATTR Encoder::onSectorReset(pcnt_unit_handle_t unit,
                                      const pcnt_watch_event_data_t *,
                                      void *user_ctx) {

  auto *state = static_cast<EncoderRuntimeState *>(user_ctx);

  portENTER_CRITICAL_ISR(&encoder_mux);
  state->currentSector = 0;

  if (state->calibrationRequested && !state->isCalibrating) {
    state->isCalibrating = true;
    state->calibrationRequested = false;
    state->completedRevolutions = 0;
    memset(state->sectorIntervals, 0, sizeof(state->sectorIntervals));
  } else if (state->isCalibrating) {
    state->completedRevolutions++;
  }
  portEXIT_CRITICAL_ISR(&encoder_mux);

  pcnt_unit_clear_count(unit);
  return false;
}

// ---------------- ISR Helpers ----------------

inline void Encoder::updatePulseTiming(EncoderRuntimeState *state,
                                       int64_t now) {

  if (state->lastPulseTimeUs != 0) {
    state->pulseIntervalUs = now - state->lastPulseTimeUs;

    if (state->isCalibrating &&
        state->completedRevolutions < MAX_CALIBRATION_REVS) {
      state
          ->sectorIntervals[state->currentSector][state->completedRevolutions] =
          state->pulseIntervalUs;
    }
  }
  state->lastPulseTimeUs = now;
}

inline void Encoder::advanceSector(EncoderRuntimeState *state) {
  state->currentSector = (state->currentSector + 1) % PULSES_PER_REV;
}

// ---------------- Speed Computation ----------------

float Encoder::computeRadPerSec() {
  return computeAngularVelocity(2.0f * static_cast<float>(M_PI));
}

float Encoder::computeRPM() { return computeAngularVelocity(60.0f); }

float Encoder::computeAngularVelocity(float scaleFactor) {
  int64_t intervalUs;
  int sector;

  taskENTER_CRITICAL(&encoder_mux);
  intervalUs = state_.pulseIntervalUs;
  sector = previousSector();
  taskEXIT_CRITICAL(&encoder_mux);

  if (intervalUs <= 0 || state_.velocityReset)
    return 0.0f;

  const float period = applySectorCorrection(sector, intervalUs);

  return scaleFactor / (PULSES_PER_REV * period);
}

float Encoder::applySectorCorrection(int sector, int64_t intervalUs) const {

  float correction = 1.0f;

  if (state_.useCorrectionLUT) {
    const int dir = state_.isDirectionInverted ? 1 : 0;
    correction = 1.0f / (state_.sectorCorrection[sector][dir] / 100.0f);
  }

  return intervalUs * correction * 1e-6f;
}

int Encoder::previousSector() const {
  return (state_.currentSector - 1 + PULSES_PER_REV) % PULSES_PER_REV;
}

void Encoder::remapCurrentSector() {
  state_.currentSector =
      (PULSES_PER_REV - state_.currentSector - 1) % PULSES_PER_REV;
}

EncoderRuntimeState &Encoder::state() { return state_; }

const EncoderRuntimeState &Encoder::state() const { return state_; }
