#pragma once

#include <stdint.h>

/**
 * @brief Inicia la task de telemetría UART.
 *
 * Transmite el estado del EKF por UART_NUM_0 a la frecuencia indicada.
 * Llamar desde main DESPUÉS de ekfTaskStart().
 *
 * @param period_ms  Periodo de transmisión en ms (recomendado: 50 → 20 Hz)
 */
void telemetryTaskStart(uint32_t period_ms);
