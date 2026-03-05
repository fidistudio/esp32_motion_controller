#include "ControlTask.h"
#include "EncoderPCNT/EncoderPCNT.h"
#include "data.h"
#include "esp_log.h"

static const char *TAG = "ControlTask";

static TaskHandle_t control_task_handle = NULL;

/* ========= Task ========= */
static void controlTask(void *arg) {
  float vel_left, vel_right;
  uint64_t i = 0;
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Actualiza los PIDs de cada rueda
    controlUpdate();

    // Obtén velocidades actuales
    vel_left = getVelocityLeft(VelocityUnits::RAD_S);
    vel_right = getVelocityRight(VelocityUnits::RAD_S);

    if (i++ % 10 == 0) {
      ESP_LOGI(TAG, "Velocidad L: %.2f RAD_S | R: %.2f RAD_S", vel_left,
               vel_right);
    }
  }
}

TaskHandle_t controlTaskStart(void) {
  xTaskCreate(controlTask, "controlTask", 3072, nullptr, 5,
              &control_task_handle);

  ESP_LOGI(TAG, "ControlTask iniciada");
  return control_task_handle;
}
