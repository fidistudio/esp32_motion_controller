#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdlib.h>
#include <string.h>

// ==========================================================
// CONSTANTS
// ==========================================================

#define TAG "HALL_ENCODER"

#define ENCODER_GPIO GPIO_NUM_23
#define SECTOR_GPIO GPIO_NUM_22

#define PULSES_PER_REV 15
#define MAX_CALIBRATION_REVS 15
#define GLITCH_FILTER_NS 10000 // 10 µs for noise rejection
#define POLL_INTERVAL_MS 200
#define PCNT_HIGH_LIMIT 2
#define PCNT_LOW_LIMIT -1

#define UART_PORT_NUM UART_NUM_0
#define UART_RX_BUF_SIZE 128

static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// ==========================================================
// TYPE DEFINITIONS
// ==========================================================

typedef struct {
  volatile int64_t last_pulse_time_us;
  volatile int64_t pulse_interval_us;
  volatile int current_sector;

  bool direction_inverted;
  bool use_lut_correction;

  bool calibration_requested;
  bool calibrating;
  int revolutions;

  // [sector][revolution] time data in microseconds
  int64_t sector_time[PULSES_PER_REV][MAX_CALIBRATION_REVS];
  float sector_correction_factor[PULSES_PER_REV][2]; // [sector][direction]
} encoder_state_t;

// ==========================================================
// GLOBAL STATE
// ==========================================================

static encoder_state_t encoder = {0};
static pcnt_unit_handle_t encoder_unit = NULL;
static pcnt_unit_handle_t sector_unit = NULL;

// ==========================================================
// INTERRUPT HANDLERS
// ==========================================================

static bool IRAM_ATTR on_encoder_pulse(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t *edata,
                                       void *user_ctx) {
  encoder_state_t *state = (encoder_state_t *)user_ctx;
  int64_t now = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&encoder_mux);
  if (state->last_pulse_time_us != 0) {
    state->pulse_interval_us = now - state->last_pulse_time_us;

    // During calibration, store timing immediately to ensure data consistency
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

static bool IRAM_ATTR on_sector_reset(pcnt_unit_handle_t unit,
                                      const pcnt_watch_event_data_t *edata,
                                      void *user_ctx) {
  encoder_state_t *state = (encoder_state_t *)user_ctx;

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

// ==========================================================
// PCNT CONFIGURATION HELPERS
// ==========================================================

static void configure_pcnt_unit(pcnt_unit_handle_t *unit) {
  pcnt_unit_config_t config = {.high_limit = PCNT_HIGH_LIMIT,
                               .low_limit = PCNT_LOW_LIMIT};
  ESP_ERROR_CHECK(pcnt_new_unit(&config, unit));
}

static void configure_pcnt_channel(pcnt_unit_handle_t unit, gpio_num_t gpio) {
  pcnt_channel_handle_t channel;
  pcnt_chan_config_t config = {.edge_gpio_num = gpio, .level_gpio_num = -1};
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &config, &channel));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
}

static void configure_glitch_filter(pcnt_unit_handle_t unit) {
  pcnt_glitch_filter_config_t filter = {.max_glitch_ns = GLITCH_FILTER_NS};
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));
}

static void start_pcnt(pcnt_unit_handle_t unit) {
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

// ==========================================================
// ENCODER SETUP
// ==========================================================

static void setup_encoder_pcnt(encoder_state_t *state) {
  configure_pcnt_unit(&encoder_unit);
  configure_pcnt_channel(encoder_unit, ENCODER_GPIO);
  configure_glitch_filter(encoder_unit);

  pcnt_event_callbacks_t callbacks = {.on_reach = on_encoder_pulse};
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(encoder_unit, &callbacks, state));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, 1));

  start_pcnt(encoder_unit);
}

static void setup_sector_pcnt(encoder_state_t *state) {
  configure_pcnt_unit(&sector_unit);
  configure_pcnt_channel(sector_unit, SECTOR_GPIO);
  configure_glitch_filter(sector_unit);

  pcnt_event_callbacks_t callbacks = {.on_reach = on_sector_reset};
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(sector_unit, &callbacks, state));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(sector_unit, 1));

  start_pcnt(sector_unit);
}

// ==========================================================
// NVS HELPERS (for LUT storage)
// ==========================================================

static void save_lut_to_nvs(const encoder_state_t *state) {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open("encoder", NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    return;
  }

  float lut[PULSES_PER_REV][2];
  size_t lut_size = sizeof(lut);
  memset(lut, 0, lut_size);

  // Load existing LUT if available (so we don’t overwrite the other direction)
  err = nvs_get_blob(nvs, "lut", lut, &lut_size);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "No previous LUT found, creating new one...");
  }

  int dir_idx = state->direction_inverted ? 1 : 0;
  for (int i = 0; i < PULSES_PER_REV; i++) {
    lut[i][dir_idx] = state->sector_correction_factor[i][dir_idx];
  }

  err = nvs_set_blob(nvs, "lut", lut, sizeof(lut));
  if (err == ESP_OK) {
    nvs_commit(nvs);
    ESP_LOGI(TAG, "LUT (%s) saved to NVS.",
             state->direction_inverted ? "INVERTED" : "NORMAL");
  } else {
    ESP_LOGE(TAG, "Failed to save LUT: %s", esp_err_to_name(err));
  }

  nvs_close(nvs);
}

static void load_lut_from_nvs(encoder_state_t *state) {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open("encoder", NVS_READONLY, &nvs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "No NVS data found (%s)", esp_err_to_name(err));
    return;
  }

  float lut[PULSES_PER_REV][2];
  size_t lut_size = sizeof(lut);
  err = nvs_get_blob(nvs, "lut", lut, &lut_size);
  if (err == ESP_OK) {
    int dir_idx = state->direction_inverted ? 1 : 0;
    for (int i = 0; i < PULSES_PER_REV; i++) {
      state->sector_correction_factor[i][dir_idx] = lut[i][dir_idx];
    }
    ESP_LOGI(TAG, "LUT (%s) loaded from NVS.",
             state->direction_inverted ? "INVERTED" : "NORMAL");
  } else {
    ESP_LOGW(TAG, "No LUT found in NVS (%s)", esp_err_to_name(err));
  }

  nvs_close(nvs);
}

// ==========================================================
// UTILITY FUNCTIONS
// ==========================================================

static float compute_rad_s(const encoder_state_t *state) {
  int64_t interval_us;
  int sector;
  float correction = 1.0f;

  taskENTER_CRITICAL(&encoder_mux);
  interval_us = state->pulse_interval_us;
  sector = (state->current_sector - 1 + PULSES_PER_REV) % PULSES_PER_REV;
  taskEXIT_CRITICAL(&encoder_mux);

  if (interval_us <= 0)
    return 0.0f;

  if (state->use_lut_correction) {
    int dir_idx = state->direction_inverted ? 1 : 0;
    correction =
        1.0f / (state->sector_correction_factor[sector][dir_idx] / 100.0f);
  }

  const float T = interval_us * correction * 1e-6f;
  return (2.0f * (float)M_PI) / (PULSES_PER_REV * T);
}

static float compute_rpm(const encoder_state_t *state) {
  int64_t interval_us;
  int sector;
  float correction = 1.0f;

  taskENTER_CRITICAL(&encoder_mux);
  interval_us = state->pulse_interval_us;
  sector = (state->current_sector - 1 + PULSES_PER_REV) % PULSES_PER_REV;
  taskEXIT_CRITICAL(&encoder_mux);

  if (interval_us <= 0)
    return 0.0f;

  if (state->use_lut_correction) {
    int dir_idx = state->direction_inverted ? 1 : 0;
    correction =
        1.0f / (state->sector_correction_factor[sector][dir_idx] / 100.0f);
  }

  const float T = interval_us * correction * 1e-6f;
  return 60.0f / (PULSES_PER_REV * T);
}

// ==========================================================
// CALIBRATION HELPERS
// ==========================================================

static float compute_robust_mean(const int64_t *samples, int n) {
  int64_t min = INT64_MAX, max = 0, sum = 0;
  for (int i = 0; i < n; i++) {
    if (samples[i] == 0)
      continue;
    if (samples[i] < min)
      min = samples[i];
    if (samples[i] > max)
      max = samples[i];
    sum += samples[i];
  }
  if (n > 2)
    sum -= (min + max);
  return (float)sum / (float)(n > 2 ? n - 2 : n);
}

static void finalize_calibration(encoder_state_t *state) {
  ESP_LOGI(TAG, "Calibration complete. Calculating LUT...");

  float global_mean = 0.0f;
  int dir_idx = state->direction_inverted ? 1 : 0;

  for (int i = 0; i < PULSES_PER_REV; i++) {
    float mean =
        compute_robust_mean(state->sector_time[i], MAX_CALIBRATION_REVS);
    state->sector_correction_factor[i][dir_idx] = mean;
    global_mean += mean;
  }

  global_mean /= PULSES_PER_REV;

  for (int i = 0; i < PULSES_PER_REV; i++) {
    state->sector_correction_factor[i][dir_idx] =
        100.0f * (state->sector_correction_factor[i][dir_idx] / global_mean);
  }

  ESP_LOGI(TAG, "Correction LUT (%% of average) [%s]:",
           state->direction_inverted ? "INVERTED" : "NORMAL");
  for (int i = 0; i < PULSES_PER_REV; i++) {
    ESP_LOGI(TAG, "Sector %02d: %.1f%%", i,
             state->sector_correction_factor[i][dir_idx]);
  }

  taskENTER_CRITICAL(&encoder_mux);
  state->calibrating = false;
  state->revolutions = 0;
  taskEXIT_CRITICAL(&encoder_mux);

  save_lut_to_nvs(state);

  ESP_LOGI(TAG, "Calibration finished.");
}

static void update_calibration(encoder_state_t *state) {
  if (!state->calibrating)
    return;

  int rev;
  taskENTER_CRITICAL(&encoder_mux);
  rev = state->revolutions;
  taskEXIT_CRITICAL(&encoder_mux);

  if (rev >= MAX_CALIBRATION_REVS) {
    finalize_calibration(state);
  }
}

// ==========================================================
// UART COMMAND PARSER
// ==========================================================

static void handle_command(char *cmd, encoder_state_t *state) {
  if (strncmp(cmd, "invert", 6) == 0) {
    char *arg = cmd + 7;
    state->direction_inverted = (strcmp(arg, "true") == 0);
    ESP_LOGI(TAG, "Direction inverted: %s",
             state->direction_inverted ? "TRUE" : "FALSE");
    load_lut_from_nvs(state);

  } else if (strncmp(cmd, "cal", 3) == 0) {
    ESP_LOGI(TAG, "Waiting for sector zero to start calibration...");
    state->calibration_requested = true;

  } else if (strncmp(cmd, "lut", 3) == 0) {
    char *arg = cmd + 4;
    state->use_lut_correction = (strcmp(arg, "true") == 0);
    ESP_LOGI(TAG, "LUT correction: %s",
             state->use_lut_correction ? "ENABLED" : "DISABLED");

  } else {
    ESP_LOGW(TAG, "Unknown command: %s", cmd);
  }
}

static void trim_newline(char *str) {
  char *p = str;
  while (*p) {
    if (*p == '\r' || *p == '\n')
      *p = '\0';
    p++;
  }
}

static void uart_command_task(void *pvParameter) {
  uint8_t data[UART_RX_BUF_SIZE];
  while (true) {
    int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1,
                              pdMS_TO_TICKS(50));
    if (len > 0) {
      data[len] = '\0';
      trim_newline((char *)data);
      handle_command((char *)data, &encoder);
    }
  }
}

// ==========================================================
// MAIN TASK
// ==========================================================

static void encoder_task(void *pvParameter) {
  setup_encoder_pcnt(&encoder);
  setup_sector_pcnt(&encoder);

  while (true) {
    if (encoder.calibrating) {
      update_calibration(&encoder);
    } else {
      float omega = compute_rad_s(&encoder);
      float rpm = compute_rpm(&encoder);

      ESP_LOGI(TAG,
               "Sector:%02d | Interval:%lld µs | ω=%.2f rad/s | %.2f RPM | "
               "Dir:%s | LUT:%s",
               encoder.current_sector, encoder.pulse_interval_us, omega, rpm,
               encoder.direction_inverted ? "INV" : "NORM",
               encoder.use_lut_correction ? "ON" : "OFF");
    }
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// ==========================================================
// ENTRY POINT
// ==========================================================

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // UART setup
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_PORT_NUM, &uart_config);

  // Load LUT for initial direction
  load_lut_from_nvs(&encoder);

  xTaskCreate(uart_command_task, "uart_command_task", 2048, NULL, 4, NULL);
  xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);
}
