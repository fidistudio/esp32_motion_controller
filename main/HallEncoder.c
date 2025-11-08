#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "HALL_ENCODER"

// === Hardware Configuration ===
#define ENCODER_GPIO GPIO_NUM_23
#define PCNT_HIGH_LIMIT 15
#define PCNT_LOW_LIMIT -1

// === Behavior Configuration ===
static volatile bool count_inverted = false; // Dynamic direction flag
#define GLITCH_FILTER_NS 10000               // 10 µs
#define POLL_INTERVAL_MS 1000                // Log update rate

// === Global Variables ===
static pcnt_unit_handle_t encoder_pcnt_unit = NULL;
static pcnt_channel_handle_t encoder_pcnt_channel = NULL;

static volatile int64_t last_pulse_time_us = 0;
static volatile int64_t pulse_interval_us = 0;
static bool isr_service_installed = false;

// === Interrupt Handler ===
static void IRAM_ATTR on_pulse_detected_isr(void *arg) {
  int64_t now = esp_timer_get_time();
  if (last_pulse_time_us != 0) {
    pulse_interval_us = now - last_pulse_time_us;
  }
  last_pulse_time_us = now;
}

// === Initialization Helpers ===
static void init_pcnt_unit(void) {
  pcnt_unit_config_t unit_config = {
      .high_limit = PCNT_HIGH_LIMIT,
      .low_limit = PCNT_LOW_LIMIT,
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &encoder_pcnt_unit));
}

static void init_pcnt_channel(void) {
  pcnt_chan_config_t channel_config = {
      .edge_gpio_num = ENCODER_GPIO,
      .level_gpio_num = -1,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(encoder_pcnt_unit, &channel_config,
                                   &encoder_pcnt_channel));

  // Always count upward — logical inversion handled in software
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      encoder_pcnt_channel, PCNT_CHANNEL_EDGE_ACTION_HOLD,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE));

  ESP_LOGI(TAG, "PCNT configured (always counting upward)");
}

static void init_glitch_filter(void) {
  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = GLITCH_FILTER_NS,
  };
  ESP_ERROR_CHECK(
      pcnt_unit_set_glitch_filter(encoder_pcnt_unit, &filter_config));
}

static void init_gpio_interrupt(void) {
  gpio_config_t io_config = {
      .pin_bit_mask = 1ULL << ENCODER_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = 1,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  gpio_config(&io_config);

  if (!isr_service_installed) {
    gpio_install_isr_service(0);
    isr_service_installed = true;
  }
  gpio_isr_handler_add(ENCODER_GPIO, on_pulse_detected_isr, NULL);
}

// === Public API ===
void encoder_set_direction(bool inverted) {
  count_inverted = inverted;
  ESP_LOGI(TAG, "Encoder direction set to: %s",
           inverted ? "INVERTED" : "NORMAL");
}

// === Main Task ===
static void PulseCounterTask(void *parameter) {
  ESP_LOGI(TAG, "Initializing pulse counter and timing task...");

  init_pcnt_unit();
  init_pcnt_channel();
  init_glitch_filter();

  ESP_ERROR_CHECK(pcnt_unit_enable(encoder_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(encoder_pcnt_unit));

  init_gpio_interrupt();

  while (true) {
    int count = 0;
    pcnt_unit_get_count(encoder_pcnt_unit, &count);

    encoder_set_direction(true);
    if (count_inverted) {
      count = -count;
    }

    float frequency_hz = 0.0f;
    if (pulse_interval_us > 0) {
      frequency_hz = 1e6f / (float)pulse_interval_us;
    }

    ESP_LOGI(TAG, "Count: %d | Interval: %lld µs | Frequency: %.2f Hz", count,
             pulse_interval_us, frequency_hz);

    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// === Entry Point ===
void app_main(void) {
  xTaskCreate(PulseCounterTask, "PulseCounterTask", 4096, NULL, 5, NULL);

}
