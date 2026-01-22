#include "timer_hw.h"

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

/* Handle del timer */
static gptimer_handle_t gptimer = NULL;

/* Handle de la task que será notificada */
static TaskHandle_t notified_task = NULL;

/* ================= ISR / CALLBACK ================= */

static bool IRAM_ATTR timer_callback(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TaskHandle_t task = (TaskHandle_t)user_data;

    if (task != NULL)
    {
        xTaskNotifyFromISR(
            task,
            0,
            eNoAction,
            &xHigherPriorityTaskWoken);
    }

    return xHigherPriorityTaskWoken == pdTRUE;
}

/* ================= START / STOP ================= */

void timer_hw_start(void)
{
    if (gptimer)
        gptimer_start(gptimer);
}

void timer_hw_stop(void)
{
    if (gptimer)
        gptimer_stop(gptimer);
}



/* ================= INIT ================= */

void timer_hw_init(TaskHandle_t task_to_notify)
{
    notified_task = task_to_notify;

    /* 1. Configuración del timer */
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000; // 1 MHz → 1 tick = 1 µs

    ESP_ERROR_CHECK(
        gptimer_new_timer(&timer_config, &gptimer));

    /* 2. Configuración de la alarma */
    gptimer_alarm_config_t alarm_config = {};
    alarm_config.alarm_count = 1000000; // 1 segundo
    alarm_config.reload_count = 0;
    alarm_config.flags = {
        .auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(
        gptimer_set_alarm_action(gptimer, &alarm_config));

    /* 3. Registrar callback */
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };

    ESP_ERROR_CHECK(
        gptimer_register_event_callbacks(
            gptimer,
            &cbs,
            (void *)notified_task));

    /* 4. Habilitar y arrancar */
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}
