#include "Encoder.h"
#include "LUTCorrection.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "HALL_ENCODER"

#define ENCODER_GPIO GPIO_NUM_23
#define SECTOR_GPIO GPIO_NUM_22

#define UART_PORT_NUM UART_NUM_0
#define UART_RX_BUF_SIZE 128
#define POLL_INTERVAL_MS 200

// ---------------- Global Objects ----------------
Encoder encoder(ENCODER_GPIO, SECTOR_GPIO);
LUTCorrection
    lut(&encoder.state_,
        "encoder"); // Cambiar nombre NVS si quieres "encoder_right", etc.

// ---------------- UART Command Parser ----------------

static void trim_newline(char *str) {
  char *p = str;
  while (*p) {
    if (*p == '\r' || *p == '\n')
      *p = '\0';
    p++;
  }
}

static void handle_command(char *cmd) {
  if (strncmp(cmd, "invert", 6) == 0) {
    char *arg = cmd + 7;
    encoder.setDirectionInverted(strcmp(arg, "true") == 0);
    ESP_LOGI(TAG, "Direction inverted: %s",
             encoder.state_.direction_inverted ? "TRUE" : "FALSE");
    lut.loadLUT();

  } else if (strncmp(cmd, "cal", 3) == 0) {
    ESP_LOGI(TAG, "Waiting for sector zero to start calibration...");
    encoder.requestCalibration();

  } else if (strncmp(cmd, "lut", 3) == 0) {
    char *arg = cmd + 4;
    encoder.state_.use_lut_correction = (strcmp(arg, "true") == 0);
    ESP_LOGI(TAG, "LUT correction: %s",
             encoder.state_.use_lut_correction ? "ENABLED" : "DISABLED");

  } else {
    ESP_LOGW(TAG, "Unknown command: %s", cmd);
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
      handle_command((char *)data);
    }
  }
}

// ---------------- Encoder Task ----------------

static void encoder_task(void *pvParameter) {
  while (true) {
    if (encoder.isCalibrating()) {
      // Check revolutions
      int rev = encoder.state_.revolutions;
      if (rev >= MAX_CALIBRATION_REVS) {
        ESP_LOGI(TAG, "Calibration complete. Calculating LUT...");
        // Compute robust mean and store LUT
        float global_mean = 0.0f;
        int dir_idx = encoder.state_.direction_inverted ? 1 : 0;

        for (int i = 0; i < PULSES_PER_REV; i++) {
          int64_t *samples = encoder.state_.sector_time[i];
          int count = 0;
          int64_t sum = 0, min_val = INT64_MAX, max_val = 0;
          for (int j = 0; j < MAX_CALIBRATION_REVS; j++) {
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
          encoder.state_.sector_correction_factor[i][dir_idx] = mean;
          global_mean += mean;
        }
        global_mean /= PULSES_PER_REV;
        for (int i = 0; i < PULSES_PER_REV; i++) {
          encoder.state_.sector_correction_factor[i][dir_idx] =
              100.0f * (encoder.state_.sector_correction_factor[i][dir_idx] /
                        global_mean);
        }

        ESP_LOGI(TAG, "Calibration finished. Saving LUT...");
        lut.saveLUT();

        encoder.state_.calibrating = false;
        encoder.state_.revolutions = 0;
      }
    } else {
      float omega = encoder.computeRadPerSec();
      float rpm = encoder.computeRPM();
      ESP_LOGI(TAG,
               "Sector:%02d | Interval:%lld µs | ω=%.2f rad/s | %.2f RPM | "
               "Dir:%s | LUT:%s",
               encoder.state_.current_sector, encoder.state_.pulse_interval_us,
               omega, rpm, encoder.state_.direction_inverted ? "INV" : "NORM",
               encoder.state_.use_lut_correction ? "ON" : "OFF");
    }
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// ---------------- Entry Point ----------------

extern "C" void app_main() {
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

  // Initialize encoder and load LUT
  encoder.begin();
  lut.loadLUT();

  // Create tasks
  xTaskCreate(uart_command_task, "uart_command_task", 2048, NULL, 4, NULL);
  xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);
}
