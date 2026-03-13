#include "UWBMeasureTask.h"
#include "UWBDriver/UWBDriver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstdint>
#include <string>

static const char *TAG = "UWBMeasureTask";

// ------------------ Configuración de antenas en el espacio ------------------
static coord Ant1{0.1f, 0.1f, -0.62f}; // x,y,z en metros
static coord Ant2{2.66f, 3.95f, -0.62f};
static coord Ant3{5.56, 0.1, -0.62f};

// ------------------ Estado publicado ------------------
static UWBState uwb_state;

// ------------------ UWBDriver ------------------
static UWBDriver uwb(UART_NUM_2, 17, 16);

// ------------------ Últimas distancias válidas (en metros) ------------------
static float last_d1 = 0.0f, last_d2 = 0.0f, last_d3 = 0.0f;

// ------------------ Función de trilateración 2D (proyectando las distancias)
// ------------------
static coord TriangularPosicion2D(float d1, float d2, float d3) {
  // Ajustar distancias proyectando al plano XY
  float h1 = std::max(0.0f, d1 * d1 - Ant1.z * Ant1.z);
  float h2 = std::max(0.0f, d2 * d2 - Ant2.z * Ant2.z);
  float h3 = std::max(0.0f, d3 * d3 - Ant3.z * Ant3.z);

  float r1 = std::sqrt(h1);
  float r2 = std::sqrt(h2);
  float r3 = std::sqrt(h3);

  // Sistema de ecuaciones
  float x1 = Ant1.x, y1 = Ant1.y;
  float x2 = Ant2.x, y2 = Ant2.y;
  float x3 = Ant3.x, y3 = Ant3.y;

  // Radios al cuadrado
  float r1_sq = r1 * r1;
  float r2_sq = r2 * r2;
  float r3_sq = r3 * r3;

  // Matriz del sistema de ecuaciones
  float A1 = 2 * (x2 - x1);
  float B1 = 2 * (y2 - y1);
  float C1 = r1_sq - r2_sq - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;

  float A2 = 2 * (x3 - x1);
  float B2 = 2 * (y3 - y1);
  float C2 = r1_sq - r3_sq - x1 * x1 + x3 * x3 - y1 * y1 + y3 * y3;

  float D = A1 * B2 - A2 * B1;

  coord pos{};
  if (D == 0.0f) {
    pos.x = pos.y = 0.0f;
  } else {
    pos.x = (C1 * B2 - C2 * B1) / D;
    pos.y = (A1 * C2 - A2 * C1) / D;
  }
  return pos;
}

// ------------------ Loop FreeRTOS ------------------
static void uwbTask(void *arg) {
  TickType_t period = pdMS_TO_TICKS(reinterpret_cast<uint32_t>(arg));

  float d1, d2, d3;

  while (true) {
    // Leer cada tag, actualizar última distancia válida
    if (uwb.getDistance("TAG00001", d1))
      last_d1 = d1;
    if (uwb.getDistance("TAG00002", d2))
      last_d2 = d2;
    if (uwb.getDistance("TAG00003", d3))
      last_d3 = d3;

    // Hacer trilateración usando últimos valores válidos
    coord p = TriangularPosicion2D(last_d1, last_d2, last_d3);

    uwb_state.x = p.x;
    uwb_state.y = p.y;
    uwb_state.valid = (last_d1 > 0.0f && last_d2 > 0.0f && last_d3 > 0.0f);
    uwb_state.timestamp_us = esp_timer_get_time();

    vTaskDelay(period);

    // ESP_LOGI(TAG, "Posicion estimada, X= %.3f, Y= %.3f", uwb_state.x,
    // uwb_state.y);
  }

  vTaskDelete(NULL);
}

// ------------------ Inicializador ------------------
void uwbMeasureTaskStart(uint32_t period_ms) {
  xTaskCreate(uwbTask, "UWBTask", 4096, (void *)period_ms, 5, NULL);
}

// ------------------ Getter de estado ------------------
const UWBState *uwbGetState() { return &uwb_state; }
