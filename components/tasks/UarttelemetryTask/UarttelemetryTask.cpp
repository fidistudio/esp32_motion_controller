#include "UarttelemetryTask.h"
#include "EKFTask/EKFTask.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "UARTTelemetry";
#define UART_PORT UART_NUM_0

static void telemetryTask(void *arg) {
  const TickType_t period = pdMS_TO_TICKS(reinterpret_cast<uint32_t>(arg));
  char buf[128];

  while (true) {
    const EKFState *state = ekfGetState();
    if (state->valid) {
      int len = snprintf(
          buf, sizeof(buf), "ODOM %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
          state->x, state->y, state->theta, state->vel_left_rads,
          state->vel_right_rads, state->pos_left_rad, state->pos_right_rad);
      uart_write_bytes(UART_PORT, buf, len);
    }
    vTaskDelay(period);
  }
  vTaskDelete(NULL);
}

void telemetryTaskStart(uint32_t period_ms) {
  xTaskCreate(telemetryTask, "TelemetryTask", 2048, (void *)period_ms, 4, NULL);
}
