#include "ControlTask.h"
#include "EncoderPCNT/EncoderPCNT.h"
#include "data.h"
#include "esp_log.h"

static const char *TAG = "ControlTask";

static TaskHandle_t control_task_handle = NULL;

/* ========= Task ========= */
static void controlTask(void *arg) {
  float vel_left, vel_right;
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Actualiza los PIDs de cada rueda
    controlUpdate();

    // Obtén velocidades actuales
    vel_left = getVelocityLeft(VelocityUnits::RAD_S);
    vel_right = getVelocityRight(VelocityUnits::RAD_S);

    ESP_LOGI(TAG, "Velocidad L: %.2f RPM | R: %.2f RPM", vel_left, vel_right);
  }
}

TaskHandle_t controlTaskStart(void) {
  xTaskCreate(controlTask, "controlTask", 3072, nullptr, 5,
              &control_task_handle);

  ESP_LOGI(TAG, "ControlTask iniciada");
  return control_task_handle;
}
