#pragma once

#include <cstdint>

/**
 * @brief Estado completo publicado por la EKFTask a 20 Hz.
 *
 * Contiene:
 *  - Pose 2D estimada por el EKF  → publicar en /odom
 *  - Posición y velocidad de cada rueda → publicar en /joint_states
 */
struct EKFState {
  /* ---- Pose 2D (EKF) ---- */
  float x;     ///< Posición X en metros
  float y;     ///< Posición Y en metros
  float theta; ///< Orientación en radianes [-π, π]

  /* ---- Rueda izquierda ---- */
  float pos_left_rad;  ///< Posición angular acumulada [rad], crece indefinido
  float vel_left_rads; ///< Velocidad angular [rad/s], con signo
  bool dir_left_fwd;   ///< true = adelante, false = reversa

  /* ---- Rueda derecha ---- */
  float pos_right_rad;
  float vel_right_rads;
  bool dir_right_fwd;

  /* ---- Meta ---- */
  int64_t timestamp_us; ///< Timestamp esp_timer_get_time()
  bool valid;           ///< true una vez que el filtro convergió
};

/**
 * @brief Inicia la EKFTask.
 *
 * Llamar desde main DESPUÉS de:
 *   - dataInit()
 *   - uwbMeasureTaskStart()
 *   - sensorMeasureTaskStart()
 *
 * @param period_ms  Periodo de predicción en ms (recomendado: 50 → 20 Hz)
 */
void ekfTaskStart(uint32_t period_ms);

/**
 * @brief Retorna puntero al estado más reciente.
 * Seguro de leer desde otras tasks en Xtensa LX6/LX7.
 */
const EKFState *ekfGetState(void);
