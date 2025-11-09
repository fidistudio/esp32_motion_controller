#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>

// ==========================================================
// CONSTANTS
// ==========================================================

#define TAG "HALL_ENCODER"

#define ENCODER_GPIO GPIO_NUM_23
#define SECTOR_GPIO GPIO_NUM_22

#define PULSES_PER_REV 15
#define MAX_CALIBRATION_REVOLUTIONS 15
#define GLITCH_FILTER_NS 10000 // 10 µs
#define POLL_INTERVAL_MS 200
#define PCNT_HIGH_LIMIT 2
#define PCNT_LOW_LIMIT -1

#define UART_PORT_NUM UART_NUM_0
#define UART_RX_BUF_SIZE 128

// ==========================================================
// TYPE DEFINITIONS
// ==========================================================

typedef struct {
  int64_t last_pulse_time_us;
  int64_t pulse_interval_us;
  int current_sector;
  bool direction_inverted;
  bool use_lut_correction;

  bool calibration_requested;
  bool calibrating;
  int revolutions;
  int sector_time[PULSES_PER_REV][MAX_CALIBRATION_REVOLUTIONS];
  float sector_correction_factor[PULSES_PER_REV];
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

  if (state->last_pulse_time_us != 0) {
    state->pulse_interval_us = now - state->last_pulse_time_us;
  }

  state->last_pulse_time_us = now;
  state->current_sector = (state->current_sector + 1) % PULSES_PER_REV;

  pcnt_unit_clear_count(unit);
  return false;
}

static bool IRAM_ATTR on_sector_reset(pcnt_unit_handle_t unit,
                                      const pcnt_watch_event_data_t *edata,
                                      void *user_ctx) {
  encoder_state_t *state = (encoder_state_t *)user_ctx;
  state->current_sector = 0;

  if (state->calibration_requested && !state->calibrating) {
    // Inicia calibración en el primer paso por sector 0
    state->calibrating = true;
    state->calibration_requested = false;
    state->revolutions = 0;
  } else if (state->calibrating) {
    state->revolutions++;
  }

  pcnt_unit_clear_count(unit);
  return false;
}

// ==========================================================
// PCNT CONFIG HELPERS
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
// UTILITY FUNCTIONS
// ==========================================================

static float compute_rad_s(const encoder_state_t *state) {
  if (state->pulse_interval_us <= 0)
    return 0.0f;
  const float T = state->pulse_interval_us * 1e-6f;
  return (2.0f * (float)M_PI) / (PULSES_PER_REV * T);
}

static float compute_rpm(const encoder_state_t *state) {
  if (state->pulse_interval_us <= 0)
    return 0.0f;
  const float T = state->pulse_interval_us * 1e-6f;
  return 60.0f / (PULSES_PER_REV * T);
}

// ==========================================================
// CALIBRATION LOGIC
// ==========================================================

static void update_calibration(encoder_state_t *state) {
  if (!state->calibrating)
    return;

  // Guardar tiempo de sector
  if (state->revolutions < MAX_CALIBRATION_REVOLUTIONS) {
    state->sector_time[state->current_sector][state->revolutions] =
        state->pulse_interval_us;
  }

  // Mostrar progreso cada vez que se completa una vuelta
  static int last_reported_rev = -1;
  if (state->revolutions != last_reported_rev &&
      state->revolutions < MAX_CALIBRATION_REVOLUTIONS) {
    ESP_LOGI(TAG, "Calibrando... vuelta %d/%d", state->revolutions + 1,
             MAX_CALIBRATION_REVOLUTIONS);
    last_reported_rev = state->revolutions;
  }

  // Calibración completa
  if (state->revolutions >= MAX_CALIBRATION_REVOLUTIONS) {
    ESP_LOGI(TAG, "Calibración completada. Calculando LUT...");

    float global_mean = 0;
    for (int i = 0; i < PULSES_PER_REV; i++) {
      float sum = 0;
      for (int r = 0; r < MAX_CALIBRATION_REVOLUTIONS; r++) {
        sum += state->sector_time[i][r];
      }
      float mean = sum / MAX_CALIBRATION_REVOLUTIONS;
      state->sector_correction_factor[i] = mean;
      global_mean += mean;
    }

    global_mean /= PULSES_PER_REV;

    // Normalizar a porcentaje
    for (int i = 0; i < PULSES_PER_REV; i++) {
      state->sector_correction_factor[i] =
          100.0f * (state->sector_correction_factor[i] / global_mean);
    }

    ESP_LOGI(TAG, "LUT de corrección (%% respecto a promedio):");
    for (int i = 0; i < PULSES_PER_REV; i++) {
      ESP_LOGI(TAG, "Sector %02d: %.1f%%", i,
               state->sector_correction_factor[i]);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    state->calibrating = false;
    state->revolutions = 0;
    ESP_LOGI(TAG, "Calibración finalizada.");
  }
}

// ==========================================================
// SERIAL COMMAND PARSER
// ==========================================================

static void handle_command(char *cmd, encoder_state_t *state) {
  if (strncmp(cmd, "invert", 6) == 0) {
    char *arg = cmd + 7;
    if (strcmp(arg, "true") == 0) {
      state->direction_inverted = true;
    } else if (strcmp(arg, "false") == 0) {
      state->direction_inverted = false;
    }
    ESP_LOGI(TAG, "Dirección invertida: %s",
             state->direction_inverted ? "TRUE" : "FALSE");

  } else if (strncmp(cmd, "cal", 3) == 0) {
    ESP_LOGI(TAG, "Esperando sector cero para iniciar calibración...");
    state->calibration_requested = true;

  } else {
    ESP_LOGW(TAG, "Comando desconocido: %s", cmd);
  }
}

static void uart_command_task(void *parameter) {
  uint8_t data[UART_RX_BUF_SIZE];
  while (true) {
    int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1,
                              pdMS_TO_TICKS(50));
    if (len > 0) {
      data[len] = '\0';
      handle_command((char *)data, &encoder);
    }
  }
}

// ==========================================================
// MAIN TASK
// ==========================================================

static void encoder_task(void *parameter) {
  setup_encoder_pcnt(&encoder);
  setup_sector_pcnt(&encoder);

  while (true) {
    if (encoder.calibrating) {
      update_calibration(&encoder);
    } else {
      // Mostrar datos sólo cuando no se calibra
      float omega = compute_rad_s(&encoder);
      float rpm = compute_rpm(&encoder);

      ESP_LOGI(
          TAG,
          "Sector:%02d | Interval:%lld µs | ω=%.2f rad/s | %.2f RPM | Dir:%s",
          encoder.current_sector, encoder.pulse_interval_us, omega, rpm,
          encoder.direction_inverted ? "INV" : "NORM");
    }
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// ==========================================================
// ENTRY POINT
// ==========================================================

void app_main(void) {
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_PORT_NUM, &uart_config);

  xTaskCreate(uart_command_task, "uart_command_task", 2048, NULL, 4, NULL);
  xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);
}
