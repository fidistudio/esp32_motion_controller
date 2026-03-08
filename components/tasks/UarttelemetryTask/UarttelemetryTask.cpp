#include "UarttelemetryTask.h"
#include "EKFTask/EKFTask.h"
#include "Telemetryframe.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "UARTTelemetry";

#define UART_PORT UART_NUM_0
#define TX_BUF_SIZE 256

/* ===============================================================
 *  Task loop
 * =============================================================== */
static void telemetryTask(void *arg) {
  const TickType_t period = pdMS_TO_TICKS(reinterpret_cast<uint32_t>(arg));

  ESP_LOGI(TAG, "UARTTelemetryTask iniciada");

  while (true) {
    const EKFState *state = ekfGetState();

    if (state->valid) {
      TelemetryFrame frame;
      frame.magic = TELEMETRY_MAGIC;
      frame.data = {
          .x = state->x,
          .y = state->y,
          .theta = state->theta,
          .pos_left = state->pos_left_rad,
          .pos_right = state->pos_right_rad,
          .vel_left = state->vel_left_rads,
          .vel_right = state->vel_right_rads,
      };
      frame.end = TELEMETRY_END_MARKER;

      uart_write_bytes(UART_PORT, reinterpret_cast<const char *>(&frame),
                       sizeof(TelemetryFrame));
    }

    vTaskDelay(period);
  }

  vTaskDelete(NULL);
}

/* ===============================================================
 *  API pública
 * =============================================================== */
void telemetryTaskStart(uint32_t period_ms) {
  xTaskCreate(telemetryTask, "TelemetryTask", 2048, (void *)period_ms, 4, NULL);
}
