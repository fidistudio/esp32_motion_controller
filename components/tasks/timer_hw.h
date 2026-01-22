#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void timer_hw_init(TaskHandle_t task_to_notify);

void timer_hw_start(void);
void timer_hw_stop(void);
