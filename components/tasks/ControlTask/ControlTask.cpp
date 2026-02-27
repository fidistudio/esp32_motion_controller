#include "ControlTask.h"
#include "EncoderPCNT/EncoderPCNT.h"
#include "data.h"
#include "esp_log.h"

static const char *TAG = "ControlTask";

static TaskHandle_t control_task_handle = NULL;

/* ========= Task ========= */
static void controlTask(void *arg) {
  float velocity;
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    controlUpdate();
    velocity = getVelocity(VelocityUnits::RAD_S);
    ESP_LOGI(TAG, "Velocidad: %f RPM", velocity);
  }
}

TaskHandle_t controlTaskStart(void) {
  xTaskCreate(controlTask, "controlTask", 3072, nullptr, 5,
              &control_task_handle);

  ESP_LOGI(TAG, "ControlTask iniciada");
  return control_task_handle;
}
