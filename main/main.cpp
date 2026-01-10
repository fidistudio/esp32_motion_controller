#include "Encoder.h"
#include "LUTCorrection.h"
#include "MotorPWM.h"
#include "hardware_config.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>
#include <cstring>

#define TAG "MAIN"

// ---------------- UART ----------------

#define UART_PORT_NUM UART_NUM_0
#define UART_RX_BUF_SIZE 128
#define POLL_INTERVAL_MS 200

// ---------------- Globals ----------------

static volatile bool stepLogging = false;
static volatile bool rampLogging = false;

// ---------------- Global Objects ----------------

static Encoder encoder(ENCODER_GPIO, SECTOR_GPIO);
static LUTCorrection lut(&encoder.state(), "encoder_left");
static MotorPWM motor(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR1_CH_A, MOTOR1_CH_B);

// ------------------------------------------------
// UART helpers
// ------------------------------------------------

static void trimNewline(char *str) {
  for (char *p = str; *p; ++p) {
    if (*p == '\r' || *p == '\n') {
      *p = '\0';
      return;
    }
  }
}

static void handleCommand(char *cmd) {

  if (strncmp(cmd, "invert", 6) == 0) {
    char *arg = cmd + 7;

    if (strcmp(arg, "true") == 0) {
      encoder.invertDirection();
    } else {
      encoder.setDirectionNormal();
    }

    lut.load();
    ESP_LOGI(TAG, "Direction inverted: %s",
             encoder.state().isDirectionInverted ? "TRUE" : "FALSE");
  }

  else if (strncmp(cmd, "cal", 3) == 0) {
    ESP_LOGI(TAG, "Waiting for sector zero to start calibration...");
    encoder.requestCalibration();
  }

  else if (strncmp(cmd, "lut", 3) == 0) {
    char *arg = cmd + 4;
    encoder.state().useCorrectionLUT = (strcmp(arg, "true") == 0);

    ESP_LOGI(TAG, "LUT correction: %s",
             encoder.state().useCorrectionLUT ? "ENABLED" : "DISABLED");
  }

  else if (strncmp(cmd, "motor1", 6) == 0) {
    float duty = 0.0f;
    if (sscanf(cmd + 7, "%f", &duty) == 1) {

      motor.setDuty(duty);

      if (motor.isStopped()) {
        encoder.resetVelocity();
      } else {
        encoder.enableVelocityTracking();
      }

      if (motor.isDirectionInverted()) {
        encoder.invertDirection();
      } else {
        encoder.setDirectionNormal();
      }

      ESP_LOGI(TAG, "Motor duty: %.2f", duty);
    } else {
      ESP_LOGW(TAG, "Invalid motor1 command: %s", cmd);
    }
  }

  else if (strncmp(cmd, "print", 5) == 0) {
    ESP_LOGI(TAG, "Printing LUT...");
    lut.print();
  }

  else if (strncmp(cmd, "step", 4) == 0) {
    stepLogging = true;
    ESP_LOGW(TAG, "STEP test started");
  }

  else if (strncmp(cmd, "ramp", 4) == 0) {
    rampLogging = true;
    ESP_LOGW(TAG, "RAMP test started");
  }

  else {
    ESP_LOGW(TAG, "Unknown command: %s", cmd);
  }
}

static void uartCommandTask(void *) {
  uint8_t data[UART_RX_BUF_SIZE];

  while (true) {
    int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1,
                              pdMS_TO_TICKS(50));

    if (len > 0) {
      data[len] = '\0';
      trimNewline(reinterpret_cast<char *>(data));
      handleCommand(reinterpret_cast<char *>(data));
    }
  }
}

// ------------------------------------------------
// Encoder / Test Task
// ------------------------------------------------

static void encoderTask(void *) {

  while (true) {

    // -------- Calibration --------
    if (encoder.isCalibrating()) {

      if (encoder.state().completedRevolutions >= MAX_CALIBRATION_REVS) {

        ESP_LOGI(TAG, "Calibration complete. Calculating LUT...");

        float globalMean = 0.0f;
        const int dir = encoder.state().isDirectionInverted ? 1 : 0;

        for (int i = 0; i < PULSES_PER_REV; i++) {
          int64_t *samples = encoder.state().sectorIntervals[i];

          int count = 0;
          int64_t sum = 0;
          int64_t minVal = INT64_MAX;
          int64_t maxVal = 0;

          for (int j = 0; j < MAX_CALIBRATION_REVS; j++) {
            const int64_t v = samples[j];
            if (v == 0)
              continue;

            minVal = (v < minVal) ? v : minVal;
            maxVal = (v > maxVal) ? v : maxVal;
            sum += v;
            count++;
          }

          if (count > 2) {
            sum -= (minVal + maxVal);
            count -= 2;
          }

          const float mean = (count > 0) ? (float)sum / (float)count : 0.0f;

          encoder.state().sectorCorrection[i][dir] = mean;
          globalMean += mean;
        }

        globalMean /= PULSES_PER_REV;

        for (int i = 0; i < PULSES_PER_REV; i++) {
          encoder.state().sectorCorrection[i][dir] =
              100.0f * (encoder.state().sectorCorrection[i][dir] / globalMean);
        }

        lut.save();

        encoder.state().isCalibrating = false;
        encoder.state().completedRevolutions = 0;

        ESP_LOGI(TAG, "Calibration finished and LUT saved");
      }
    }

    // -------- STEP test --------
    else if (stepLogging) {

      const int64_t TOTAL_US = 5'000'000;
      const int64_t ZERO_US = 1'000'000;
      const int64_t t0 = esp_timer_get_time();

      motor.setDuty(0.0f);
      encoder.resetVelocity();

      printf("# BEGIN_STEP\n");
      printf("# t_ms, duty, vel_rad_s\n");

      float lastDuty = 0.0f;

      while ((esp_timer_get_time() - t0) < TOTAL_US) {
        const int64_t elapsed = esp_timer_get_time() - t0;
        const float duty = (elapsed < ZERO_US) ? 0.0f : 1.0f;

        if (duty != lastDuty) {
          motor.setDuty(duty);
          encoder.enableVelocityTracking();
          lastDuty = duty;
        }

        printf("%lld, %.3f, %.4f\n", (long long)(elapsed / 1000), duty,
               encoder.computeRadPerSec());

        vTaskDelay(pdMS_TO_TICKS(10));
      }

      printf("END_STEP\n");

      motor.setDuty(0.0f);
      encoder.resetVelocity();
      stepLogging = false;
    }

    // -------- RAMP test --------
    else if (rampLogging) {

      const int64_t TOTAL_US = 5'000'000;
      const int64_t t0 = esp_timer_get_time();

      motor.setDuty(0.0f);
      encoder.resetVelocity();

      printf("# BEGIN_RAMP\n");
      printf("# t_ms, duty, vel_rad_s\n");

      float lastDuty = 0.0f;

      while ((esp_timer_get_time() - t0) < TOTAL_US) {
        const int64_t elapsed = esp_timer_get_time() - t0;
        float duty = (float)elapsed / (float)TOTAL_US;
        if (duty > 1.0f)
          duty = 1.0f;

        if (duty != lastDuty) {
          motor.setDuty(duty);
          encoder.enableVelocityTracking();
          lastDuty = duty;
        }

        printf("%lld, %.3f, %.4f\n", (long long)(elapsed / 1000), duty,
               encoder.computeRadPerSec());

        vTaskDelay(pdMS_TO_TICKS(10));
      }

      printf("END_RAMP\n");

      motor.setDuty(0.0f);
      encoder.resetVelocity();
      rampLogging = false;
    }

    // -------- Normal logging --------
    else {
      ESP_LOGI(TAG,
               "Sector:%02d | Interval:%lld µs | ω=%.2f rad/s | %.2f RPM | "
               "Dir:%s | LUT:%s",
               encoder.state().currentSector, encoder.state().pulseIntervalUs,
               encoder.computeRadPerSec(), encoder.computeRPM(),
               encoder.state().isDirectionInverted ? "INV" : "NORM",
               encoder.state().useCorrectionLUT ? "ON" : "OFF");
    }

    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// ------------------------------------------------
// app_main
// ------------------------------------------------

extern "C" void app_main() {

  ESP_ERROR_CHECK(nvs_flash_init());

  const uart_config_t uartConfig = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE * 2, 0, 0, nullptr, 0);

  uart_param_config(UART_PORT_NUM, &uartConfig);

  encoder.begin();
  lut.load();
  motor.begin();

  xTaskCreate(uartCommandTask, "uart_command_task", 4096, nullptr, 4, nullptr);

  xTaskCreate(encoderTask, "encoder_task", 4096, nullptr, 5, nullptr);
}
