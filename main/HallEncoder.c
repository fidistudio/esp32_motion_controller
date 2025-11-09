#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

// ==========================================================
// Configuration
// ==========================================================

#define TAG "HALL_ENCODER"

#define ENCODER_GPIO GPIO_NUM_23
#define SECTOR_GPIO GPIO_NUM_22
#define PULSES_PER_REV 15
#define GLITCH_FILTER_NS 10000 // 10 µs
#define POLL_INTERVAL_MS 1000
#define PCNT_HIGH_LIMIT 2
#define PCNT_LOW_LIMIT -1

// ==========================================================
// Global State
// ==========================================================

static pcnt_unit_handle_t encoder_unit = NULL;
static pcnt_channel_handle_t encoder_channel = NULL;
static pcnt_unit_handle_t sector_unit = NULL;
static pcnt_channel_handle_t sector_channel = NULL;

static volatile int64_t last_pulse_time_us = 0;
static volatile int64_t pulse_interval_us = 0;
static volatile int sector = 0;
static volatile bool direction_inverted = true;

// ==========================================================
// ISR: PCNT #1 (velocidad angular)
// ==========================================================

static bool IRAM_ATTR on_encoder_pulse(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t *edata,
                                       void *user_ctx) {
  int64_t now = esp_timer_get_time();

  if (last_pulse_time_us != 0) {
    pulse_interval_us = now - last_pulse_time_us;
  }
  last_pulse_time_us = now;

  // Reset count for next pulse
  pcnt_unit_clear_count(unit);

  // Avanza sector
  sector = (sector + 1) % PULSES_PER_REV;

  return false;
}

// ==========================================================
// ISR: PCNT #2 (reseteo de sector)
// ==========================================================

static bool IRAM_ATTR on_sector_reset(pcnt_unit_handle_t unit,
                                      const pcnt_watch_event_data_t *edata,
                                      void *user_ctx) {
  sector = 0;                  // Reinicia el sector
  pcnt_unit_clear_count(unit); // Limpia el contador de este PCNT
  return false;
}

// ==========================================================
// Common PCNT setup helpers
// ==========================================================

static void configure_pcnt_unit(pcnt_unit_handle_t *unit) {
  pcnt_unit_config_t config = {
      .high_limit = PCNT_HIGH_LIMIT,
      .low_limit = PCNT_LOW_LIMIT,
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&config, unit));
}

static void configure_pcnt_channel(pcnt_unit_handle_t unit,
                                   pcnt_channel_handle_t *channel,
                                   gpio_num_t gpio) {
  pcnt_chan_config_t config = {
      .edge_gpio_num = gpio,
      .level_gpio_num = -1,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &config, channel));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(*channel, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
}

static void configure_glitch_filter(pcnt_unit_handle_t unit) {
  pcnt_glitch_filter_config_t filter = {
      .max_glitch_ns = GLITCH_FILTER_NS,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));
}

static void start_pcnt_unit(pcnt_unit_handle_t unit) {
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

// ==========================================================
// PCNT #1 setup (encoder principal)
// ==========================================================

static void setup_encoder_pcnt(void) {
  configure_pcnt_unit(&encoder_unit);
  configure_pcnt_channel(encoder_unit, &encoder_channel, ENCODER_GPIO);
  configure_glitch_filter(encoder_unit);

  pcnt_event_callbacks_t callbacks = {
      .on_reach = on_encoder_pulse,
  };
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(encoder_unit, &callbacks, NULL));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, 1));

  start_pcnt_unit(encoder_unit);
}

// ==========================================================
// PCNT #2 setup (reseteo de sector)
// ==========================================================

static void setup_sector_pcnt(void) {
  configure_pcnt_unit(&sector_unit);
  configure_pcnt_channel(sector_unit, &sector_channel, SECTOR_GPIO);
  configure_glitch_filter(sector_unit);

  pcnt_event_callbacks_t callbacks = {
      .on_reach = on_sector_reset,
  };
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(sector_unit, &callbacks, NULL));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(sector_unit, 1));

  start_pcnt_unit(sector_unit);
}

// ==========================================================
// Task principal
// ==========================================================

static void pulse_counter_task(void *parameter) {
  ESP_LOGI(TAG, "Inicializando encoders...");

  setup_encoder_pcnt();
  setup_sector_pcnt();

  while (true) {
    float angular_speed_rad_s = 0.0f;
    float angular_speed_rpm = 0.0f;

    if (pulse_interval_us > 0) {
      const float us_to_s = 1e-6f;
      float pulse_period_s = pulse_interval_us * us_to_s;
      angular_speed_rad_s =
          (2.0f * (float)M_PI) / (PULSES_PER_REV * pulse_period_s);
      angular_speed_rpm = (60.0f / (PULSES_PER_REV * pulse_period_s));
    }

    ESP_LOGI(TAG,
             "Sector: %02d | Interval: %lld µs | ω = %.2f rad/s | %.2f RPM | "
             "Dir: %s",
             sector, pulse_interval_us, angular_speed_rad_s, angular_speed_rpm,
             direction_inverted ? "INV" : "NORM");

    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// ==========================================================
// Entry Point
// ==========================================================

void app_main(void) {
  xTaskCreate(pulse_counter_task, "pulse_counter_task", 4096, NULL, 5, NULL);
}
