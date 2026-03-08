#pragma once

#include <cstdint>

/**
 * @brief Estado de pose publicado por la EKFTask.
 *
 * Actualizado a 20 Hz (predicción) y corregido a ~1 Hz (UWB)
 * y siempre que haya nueva medición IMU disponible.
 */
struct EKFState {
  float x;              ///< Posición X en metros
  float y;              ///< Posición Y en metros
  float theta;          ///< Orientación en radianes [-π, π]
  int64_t timestamp_us; ///< Timestamp esp_timer_get_time()
  bool valid;           ///< true una vez que el filtro convergió
};

/**
 * @brief Inicia la EKFTask.
 *
 * Llamar a esta función desde main DESPUÉS de:
 *   - dataInit()
 *   - uwbMeasureTaskStart()
 *   - sensorMeasureTaskStart()
 *
 * @param period_ms  Periodo de predicción en ms (recomendado: 50 → 20 Hz)
 */
void ekfTaskStart(uint32_t period_ms);

/**
 * @brief Retorna puntero al estado EKF más reciente.
 *
 * Seguro de leer desde otras tasks (escritura atómica por campo float
 * en Xtensa LX6/LX7 con alineación natural).
 */
const EKFState *ekfGetState(void);
