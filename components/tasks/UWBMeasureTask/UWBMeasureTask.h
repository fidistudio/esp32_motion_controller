#pragma once

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Coordenadas de un punto en 3D
 */
struct coord {
  float x;
  float y;
  float z;
};

/**
 * @brief Estado publicado por la task UWB
 */
struct UWBState {
  float x; // posición proyectada en XY
  float y;
  bool valid; // true si todas las distancias son válidas
  uint64_t timestamp_us;
};

/**
 * @brief Inicia la task de lectura UWB y trilateración
 * @param period_ms Periodo de lectura en ms (default 1000ms = 1 Hz)
 */
void uwbMeasureTaskStart(uint32_t period_ms = 1000);

/**
 * @brief Retorna un puntero al estado más reciente
 */
const UWBState *uwbGetState();
