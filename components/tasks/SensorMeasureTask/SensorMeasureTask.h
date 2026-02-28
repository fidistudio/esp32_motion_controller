#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Inicia la task que lee los sensores (IMU por ahora)
 * @return TaskHandle_t handle de la task creada
 */
TaskHandle_t sensorMeasureTaskStart(void);
