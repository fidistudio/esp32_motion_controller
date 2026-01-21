#include "timer_task.h"
#include "esp_log.h"

static const char *TAG = "TIMER_TASK";

static TaskHandle_t timer_task_handle = NULL;

static void timer_task(void *arg)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "oal ya desperte");
    }
}

TaskHandle_t timer_task_start(void)
{
    xTaskCreate(
        timer_task,
        "timer_task",
        2048,
        NULL,
        5,
        &timer_task_handle
    );

    return timer_task_handle;
}
