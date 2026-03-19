#include "EncoderPCNT.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "portmacro.h"
static const char *TAG = "EncoderPCNT";

/* ================= PCNT ================= */

PCNTPulse::PCNTPulse(gpio_num_t pulse_gpio, isr_cb_t callback,
                     void *callback_arg, uint32_t glitch_filter_ns) {
  ESP_LOGI("PCNTPulse", "Inicializando PCNT en GPIO %d", pulse_gpio);
  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.low_limit = -10;
  unit_cfg.high_limit = 10;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit_));

  pcnt_chan_config_t chan_cfg = {};
  chan_cfg.edge_gpio_num = pulse_gpio;
  chan_cfg.level_gpio_num = -1;
  ESP_ERROR_CHECK(pcnt_new_channel(unit_, &chan_cfg, &channel_));

  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel_, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));

  if (glitch_filter_ns > 0) {
    pcnt_glitch_filter_config_t filter_cfg = {};
    filter_cfg.max_glitch_ns = glitch_filter_ns;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit_, &filter_cfg));
  }

  if (callback) {
    pcnt_event_callbacks_t cbs = {};
    cbs.on_reach = callback;
    ESP_ERROR_CHECK(
        pcnt_unit_register_event_callbacks(unit_, &cbs, callback_arg));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_, 1));
  }

  ESP_ERROR_CHECK(pcnt_unit_enable(unit_));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit_));
  ESP_ERROR_CHECK(pcnt_unit_start(unit_));
}

void PCNTPulse::clearCount() { pcnt_unit_clear_count(unit_); }

/* ================= EncoderPulseSource ================= */

EncoderPulseSource::EncoderPulseSource(gpio_num_t pulse_gpio,
                                       gpio_num_t sector0_gpio,
                                       QueueHandle_t queue, int8_t num_sectors,
                                       uint32_t glitch_filter_ns)
    : queue_(queue), sector_(0), inverted_(false), enabled_(false),
      last_time_us_(0), NUM_SECTORS_(num_sectors),
      pulse_pcnt_(pulse_gpio, onPulse, this, glitch_filter_ns),
      sector0_pcnt_(sector0_gpio, onSector0, this, glitch_filter_ns) {}

void EncoderPulseSource::setInverted(bool inverted) {
  if (inverted != inverted_) {
    portENTER_CRITICAL(&pulse_mux_);
    sector_ = (NUM_SECTORS_ - sector_) % NUM_SECTORS_;
    inverted_ = inverted;
    portEXIT_CRITICAL(&pulse_mux_);
  }
}

void EncoderPulseSource::setEnabled(bool enabled) {
  portENTER_CRITICAL(&pulse_mux_);
  enabled_ = enabled;
  portEXIT_CRITICAL(&pulse_mux_);
}
void EncoderPulseSource::enableForCalibration() {
  portENTER_CRITICAL(&pulse_mux_);
  enabled_ = true;
  portEXIT_CRITICAL(&pulse_mux_);
}

bool IRAM_ATTR EncoderPulseSource::onPulse(pcnt_unit_handle_t,
                                           const pcnt_watch_event_data_t *,
                                           void *arg) {
  auto *self = static_cast<EncoderPulseSource *>(arg);

  // esp_rom_printf("Pulso\n");

  portENTER_CRITICAL_ISR(&self->pulse_mux_);
  bool enabled = self->enabled_;
  portEXIT_CRITICAL_ISR(&self->pulse_mux_);

  if (!enabled) {
    self->pulse_pcnt_.clearCount();
    return false;
  }

  int64_t now = esp_timer_get_time();
  int64_t dt = now - self->last_time_us_;
  self->last_time_us_ = now;

  portENTER_CRITICAL_ISR(&self->pulse_mux_);
  self->sector_ = (self->sector_ + 1) % self->NUM_SECTORS_;
  portEXIT_CRITICAL_ISR(&self->pulse_mux_);

  EncoderSample sample{.sector = self->sector_, .dt_us = dt};

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueOverwriteFromISR(self->queue_, &sample, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR();

  self->pulse_pcnt_.clearCount();
  return false;
}

bool IRAM_ATTR EncoderPulseSource::onSector0(pcnt_unit_handle_t,
                                             const pcnt_watch_event_data_t *,
                                             void *arg) {
  auto *self = static_cast<EncoderPulseSource *>(arg);

  // esp_rom_printf("Reset\n");
  portENTER_CRITICAL_ISR(&self->pulse_mux_);
  self->sector_ = 0;
  portEXIT_CRITICAL_ISR(&self->pulse_mux_);
  self->sector0_pcnt_.clearCount();
  return false;
}

/* ================= Encoder ================= */

Encoder::Encoder(gpio_num_t pulse_pin, gpio_num_t sector_pin, float (*lut)[2],
                 uint32_t glitch_filter_ns, BaseType_t core_id)
    : lut_(lut), calibrating_(false), calibration_requested_(false),
      inverted_(false), sector_(0), dt_(0), revs_(0), position_rad_(0.0f),
      core_id_(core_id), input_queue_(xQueueCreate(1, sizeof(EncoderSample))),
      pulse_source_(pulse_pin, sector_pin, input_queue_, NUM_SECTORS_,
                    glitch_filter_ns),
      done_task_(nullptr) {
  xTaskCreatePinnedToCore(Encoder::taskEntry, "EncoderTask", 4096, this, 6,
                          &task_handle_, core_id);
}

/* ================= Calibración (sin cambios) ================= */

void Encoder::startCalibration() {
  calibration_requested_ = true;
  revs_ = 0;
}
void Encoder::stopCalibration() {
  calibration_requested_ = false;
  calibrating_ = false;
}
bool Encoder::isCalibrating() { return calibrating_ || calibration_requested_; }
void Encoder::setInverted(bool i) {
  inverted_ = i;
  pulse_source_.setInverted(i);
}
void Encoder::setEnabled(bool enabled) { pulse_source_.setEnabled(enabled); }
void Encoder::enableForCalibration() { pulse_source_.enableForCalibration(); }
bool Encoder::isInverted() { return inverted_; }

/* ================= Task loop ================= */

void Encoder::taskEntry(void *arg) { static_cast<Encoder *>(arg)->taskLoop(); }

void Encoder::taskLoop() {
  EncoderSample sample;

  while (true) {
    if (xQueueReceive(input_queue_, &sample, portMAX_DELAY) == pdTRUE) {
      if (isCalibrating()) {
        calibrateStep(sample);
      } else {
        portENTER_CRITICAL(&mux_);
        sector_ = sample.sector;
        dt_ = sample.dt_us;

        // Acumular posición angular con signo según dirección
        // inverted_ == false → adelante (+), true → reversa (-)
        position_rad_ += inverted_ ? -RAD_PER_PULSE_ : RAD_PER_PULSE_;

        portEXIT_CRITICAL(&mux_);

        // esp_rom_printf("sector=%d dt=%lld us\n", sample.sector,
        // sample.dt_us);
      }
    }
  }
}

/* ================= Calibración (sin cambios respecto al original)
 * ================= */

void Encoder::calibrateStep(const EncoderSample &sample) {
  int64_t dt = sample.dt_us;
  int8_t sector = (sample.sector - 1 + NUM_SECTORS_) % NUM_SECTORS_;

  if (sector == 0) {
    if (calibration_requested_ && !calibrating_) {
      calibrating_ = true;
      calibration_requested_ = false;
    } else
      revs_++;
  }

  if (calibrating_) {
    if (revs_ < MAX_CALIBRATION_REVS_) {
      sector_time[sector][revs_] = dt;
      ESP_LOGI(TAG, "Sector: %d, revs: %d, dt: %d", sector, revs_, (int)dt);
    } else {
      float global_mean = 0.0f;

      for (int i = 0; i < NUM_SECTORS_; i++) {
        int64_t *samples = sector_time[i];
        int count = 0;
        int64_t sum = 0, min_val = INT64_MAX, max_val = 0;

        for (int j = 0; j < MAX_CALIBRATION_REVS_; j++) {
          int64_t v = samples[j];
          if (v == 0)
            continue;
          if (v < min_val)
            min_val = v;
          if (v > max_val)
            max_val = v;
          sum += v;
          count++;
        }
        if (count > 2)
          sum -= (min_val + max_val);

        float mean =
            (count > 2) ? ((float)sum / (float)(count - 2)) : ((float)sum);

        lut_[i][inverted_] = mean;
        global_mean += mean;
      }

      global_mean /= NUM_SECTORS_;
      for (int i = 0; i < NUM_SECTORS_; i++)
        lut_[i][inverted_] = 100.0f * (lut_[i][inverted_] / global_mean);

      calibrating_ = false;
      if (done_task_ != nullptr)
        xTaskNotifyGive(done_task_);
    }
  }
}

/* ================= Getters de velocidad y posición ================= */

float Encoder::getVelocity(VelocityUnits units) {
  int64_t dt;
  int8_t sector;

  portENTER_CRITICAL(&mux_);
  dt = dt_;
  sector = (sector_ - 1 + NUM_SECTORS_) % NUM_SECTORS_;
  portEXIT_CRITICAL(&mux_);

  float correction = 1.0f / (lut_[sector][inverted_] / 100.0f);
  int8_t sign = inverted_ ? -1 : 1;
  float T = dt * correction * 1e-6f;

  if (T <= 0.0f)
    return 0.0f;

  if (units == VelocityUnits::RPM)
    return (sign * 60.0f) / (NUM_SECTORS_ * T);
  else
    return (sign * 2.0f * static_cast<float>(M_PI)) / (NUM_SECTORS_ * T);
}

float Encoder::getPosition() {
  float pos;
  portENTER_CRITICAL(&mux_);
  pos = position_rad_;
  portEXIT_CRITICAL(&mux_);
  return pos;
}

void Encoder::resetPosition() {
  portENTER_CRITICAL(&mux_);
  position_rad_ = 0.0f;
  portEXIT_CRITICAL(&mux_);
}

/* ================= Misc ================= */

void Encoder::setDoneCallback(TaskHandle_t task) { done_task_ = task; }

void Encoder::clearDT() {
  portENTER_CRITICAL(&mux_);
  dt_ = 0;
  portEXIT_CRITICAL(&mux_);
}
