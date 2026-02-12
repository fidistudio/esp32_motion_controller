#include "EncoderPCNT.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h>

#include "esp_log.h"
static const char *TAG = "EncoderPCNT";

/* ================= PCNT ================= */

PCNTPulse::PCNTPulse(gpio_num_t pulse_gpio,
                     isr_cb_t callback, void *callback_arg,
                     uint32_t glitch_filter_ns)
{
    // 1. Crear unidad PCNT
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.low_limit = -10;
    unit_cfg.high_limit = 10;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit_));

    // 2. Crear canal
    pcnt_chan_config_t chan_cfg = {};
    chan_cfg.edge_gpio_num = pulse_gpio;
    chan_cfg.level_gpio_num = -1;

    ESP_ERROR_CHECK(pcnt_new_channel(unit_, &chan_cfg, &channel_));

    // 3. Contar solo flanco de subida
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            channel_,
            PCNT_CHANNEL_EDGE_ACTION_HOLD,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    // 4. Filtro de glitches
    if (glitch_filter_ns > 0)
    {
        pcnt_glitch_filter_config_t filter_cfg = {};
        filter_cfg.max_glitch_ns = glitch_filter_ns;
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit_, &filter_cfg));
    }

    // 5. Registrar callback
    if (callback)
    {
        pcnt_event_callbacks_t cbs = {};
        cbs.on_reach = callback;

        ESP_ERROR_CHECK(
            pcnt_unit_register_event_callbacks(unit_, &cbs, callback_arg));

        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_, 1));
    }

    // 6. Activar PCNT
    ESP_ERROR_CHECK(pcnt_unit_enable(unit_));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit_));
    ESP_ERROR_CHECK(pcnt_unit_start(unit_));
}

void PCNTPulse::clearCount()
{
    pcnt_unit_clear_count(unit_);
}

/* ================= EncoderPulseSource ================= */

EncoderPulseSource::EncoderPulseSource(gpio_num_t pulse_gpio,
                                       gpio_num_t sector0_gpio,
                                       QueueHandle_t queue,
                                       int8_t num_sectors,
                                       uint32_t glitch_filter_ns)
    : queue_(queue), sector_(0), inverted_(false), last_time_us_(0), NUM_SECTORS_(num_sectors),
      pulse_pcnt_(pulse_gpio, onPulse, this, glitch_filter_ns),
      sector0_pcnt_(sector0_gpio, onSector0, this, glitch_filter_ns)
{
}

void EncoderPulseSource::setInverted(bool inverted)
{
    if (inverted != inverted_)
    {
        sector_ = (NUM_SECTORS_ - sector_ - 1) % NUM_SECTORS_;
        inverted_ = inverted;
    }
}

bool IRAM_ATTR EncoderPulseSource::onPulse(
    pcnt_unit_handle_t,
    const pcnt_watch_event_data_t *,
    void *arg)
{
    auto *self = static_cast<EncoderPulseSource *>(arg);

    int64_t now = esp_timer_get_time();
    int64_t dt = now - self->last_time_us_;
    self->last_time_us_ = now;

    self->sector_ = (self->sector_ + 1) % self->NUM_SECTORS_;

    EncoderSample sample{
        .sector = self->sector_,
        .dt_us = dt};

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueOverwriteFromISR(self->queue_, &sample, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }

    self->pulse_pcnt_.clearCount();
    return false;
}

bool IRAM_ATTR EncoderPulseSource::onSector0(
    pcnt_unit_handle_t,
    const pcnt_watch_event_data_t *,
    void *arg)
{
    auto *self = static_cast<EncoderPulseSource *>(arg);
    self->sector_ = 0;

    self->sector0_pcnt_.clearCount();
    return false;
}

/* ================= Encoder ================= */

portMUX_TYPE Encoder::mux_ = portMUX_INITIALIZER_UNLOCKED;

Encoder::Encoder(
    gpio_num_t pulse_pin,
    gpio_num_t sector_pin,
    float (*lut)[2],
    uint32_t glitch_filter_ns)
    : lut_(lut),
      calibrating_(false),
      calibration_requested_(false),
      inverted_(false),
      sector_(0),
      dt_(0),
      revs_(0),
      input_queue_(xQueueCreate(1, sizeof(EncoderSample))),
      pulse_source_(
          pulse_pin,
          sector_pin,
          input_queue_,
          NUM_SECTORS_,
          glitch_filter_ns),
      done_task_(nullptr)
{
    xTaskCreatePinnedToCore(
        Encoder::taskEntry,
        "EncoderTask",
        4096,
        this,
        10,
        &task_handle_,
        tskNO_AFFINITY);
}

void Encoder::startCalibration()
{
    calibration_requested_ = true;
    revs_ = 0;
}

void Encoder::stopCalibration()
{
    calibration_requested_ = false;
    calibrating_ = false;
}

bool Encoder::isCalibrating()
{
    return calibrating_ || calibration_requested_;
}

void Encoder::setInverted(bool inverted)
{
    inverted_ = inverted;
    pulse_source_.setInverted(inverted_);
}

bool Encoder::isInverted()
{
    return inverted_;
}

void Encoder::taskEntry(void *arg)
{
    static_cast<Encoder *>(arg)->taskLoop();
}

void Encoder::taskLoop()
{
    EncoderSample sample;

    while (true)
    {
        if (xQueueReceive(input_queue_, &sample, portMAX_DELAY) == pdTRUE)
        {
            if (isCalibrating())
            {
                calibrateStep(sample);
            }
            else
            {
                portENTER_CRITICAL(&mux_);
                sector_ = sample.sector;
                dt_ = sample.dt_us;
                portEXIT_CRITICAL(&mux_);
            }
        }
    }
}

void Encoder::calibrateStep(const EncoderSample &sample)
{
    int64_t dt = sample.dt_us;
    int8_t sector = sample.sector;

    if (sector == 0)
    {
        if (calibration_requested_ && !calibrating_)
        {
            calibrating_ = true;
            calibration_requested_ = false;
        }
        else
            revs_++;
    }
    if (calibrating_)
    {
        if (revs_ < MAX_CALIBRATION_REVS_)
        {
            sector_time[sector][revs_] = dt;
            ESP_LOGI(TAG, "Sector: %d, revs: %d, dt: %d", sector, revs_, dt);
        }
        else
        {

            float global_mean = 0.0f;

            for (int i = 0; i < NUM_SECTORS_; i++)
            {
                int64_t *samples = sector_time[i];
                int count = 0;
                int64_t sum = 0, min_val = INT64_MAX, max_val = 0;
                for (int j = 0; j < MAX_CALIBRATION_REVS_; j++)
                {
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
            {
                lut_[i][inverted_] =
                    100.0f * (lut_[i][inverted_] /
                              global_mean);
            }

            calibrating_ = false;
            if (done_task_ != nullptr)
                xTaskNotifyGive(done_task_);
        }
    }
}

float Encoder::getVelocity(VelocityUnits units)
{
    int64_t dt;
    int8_t sector;

    portENTER_CRITICAL(&mux_);
    dt = dt_;
    sector = sector_;
    portEXIT_CRITICAL(&mux_);
    float correction = 1.0f / (lut_[sector][inverted_] / 100.0f);
    int8_t sign = inverted_ ? -1 : 1;
    float T = dt * correction * 1e-6f;

    // ESP_LOGI(TAG, "Sector: %d, dt: %d, lut:%f, correction: %f, T: %f", sector, dt, lut_[sector][inverted_], correction, T);

    if (T <= 0.0f)
    {
        return 0.0f;
    }

    if (units == VelocityUnits::RPM)
    {
        return (sign * 60.0f) / (NUM_SECTORS_ * T);
    }
    else
    {
        return (sign * 2.0f * static_cast<float>(M_PI)) / (NUM_SECTORS_ * T);
    }
}

void Encoder::setDoneCallback(TaskHandle_t task)
{
    done_task_ = task;
}