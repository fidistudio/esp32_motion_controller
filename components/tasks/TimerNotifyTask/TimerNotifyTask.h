#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void timer_notify_init(TaskHandle_t task_to_notify);

void timer_notify_start(void);
void timer_notify_stop(void);

bool is_timer_enabled(void);