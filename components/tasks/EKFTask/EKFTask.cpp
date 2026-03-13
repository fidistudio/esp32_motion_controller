#include "EKFTask.h"

#include "EKF/EKF.h"
#include "SensorMeasureTask/SensorMeasureTask.h"
#include "UWBMeasureTask/UWBMeasureTask.h"
#include "data.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include <cmath>

static const char *TAG = "EKFTask";

/* ===============================================================
 *  Parámetros físicos del robot — sin cambios
 * =============================================================== */
static constexpr float WHEEL_RADIUS_L = 0.31f / 2.0f;
static constexpr float WHEEL_RADIUS_R = 0.31f / 2.0f;
static constexpr float HALF_BASE = 0.37f / 2.0f;

/* ===============================================================
 *  Parámetros del filtro
 *
 *  R_IMU  bajo  → mucha confianza en la IMU (señal limpia, sin mag)
 *  R_ODOM alto  → poca confianza en odometría (slip posible)
 *                 pero suficiente para corregir drift de largo plazo
 *
 *  Guía de tuning:
 *    - Si el robot gira y el yaw tarda en seguirlo → bajar R_IMU
 *    - Si el yaw deriva con el tiempo en línea recta → bajar R_ODOM
 *    - Si el yaw salta al arrancar los motores → subir R_IMU
 * =============================================================== */
static constexpr float DT_S = 0.05f;
static constexpr float Q_XY = 0.001f;
static constexpr float Q_THETA = 0.0005f;
static constexpr float R_UWB = 0.04f;
static constexpr float R_IMU = 0.002f; // subido ligeramente: el yaw ahora
                                       // deriva lento en vez de saltar
static constexpr float R_ODOM = 0.05f; // NUEVO: ~13° de incertidumbre 1σ

/* ===============================================================
 *  Estado publicado
 * =============================================================== */
static EKFState ekf_state;

/* ===============================================================
 *  Helpers: normalización angular
 * =============================================================== */
static inline float norm_angle(float a) {
  while (a > M_PI)
    a -= 2.f * M_PI;
  while (a < -M_PI)
    a += 2.f * M_PI;
  return a;
}

/* ===============================================================
 *  Loop principal
 * =============================================================== */
static void ekfTask(void *arg) {
  const TickType_t period = pdMS_TO_TICKS(reinterpret_cast<uint32_t>(arg));

  // CAMBIO: constructor con 6 parámetros (añadido R_ODOM)
  EKF ekf(DT_S, Q_XY, Q_THETA, R_UWB, R_IMU, R_ODOM);

  /* ---- Esperar primera medición UWB válida ---- */
  ESP_LOGI(TAG, "Esperando primera medicion UWB...");

  const UWBState *uwb = uwbGetState();
  while (!uwb->valid) {
    vTaskDelay(pdMS_TO_TICKS(100));
    uwb = uwbGetState();
  }

  /* ---- Inicializar con posición real ---- */
  const IMUState *imu = imuGetState();
  float theta0 = imu->valid ? imu->yaw_rad : 0.0f;
  ekf.reset(uwb->x, uwb->y, theta0);

  ESP_LOGI(TAG, "EKF inicializado en x=%.3f y=%.3f theta=%.3f rad", uwb->x,
           uwb->y, theta0);

  int64_t last_uwb_ts = uwb->timestamp_us;

  /* ---- Estado interno de odometría ---- */
  // NUEVO: acumulador de yaw por encoders, inicializado con el mismo
  // theta0 para que IMU y odometría partan del mismo frame de referencia
  float odom_yaw = theta0;
  float prev_pos_l = getPositionLeft(); // posición en radianes de rueda
  float prev_pos_r = getPositionRight();

  /* ---- Loop principal ---- */
  while (true) {

    /* 1. Odometría diferencial → v, ω para el predict */
    float omega_l, omega_r, pos_l, pos_r;
    getWheelSnapshot(&omega_l, &omega_r, &pos_l, &pos_r);

    float v = (WHEEL_RADIUS_R * omega_r + WHEEL_RADIUS_L * omega_l) / 2.0f;
    float omega = (WHEEL_RADIUS_R * omega_r - WHEEL_RADIUS_L * omega_l) /
                  (2.0f * HALF_BASE);

    /* 2. Acumulación de yaw por odometría (NUEVO)
     *
     * Se usa posición de encoder (no velocidad) para acumular el ángulo.
     * Esto evita que pequeños errores de velocidad se integren dos veces
     * (una en predict y otra aquí). La diferencia de posición es la fuente
     * más directa y menos ruidosa del desplazamiento angular.
     */

    float dl =
        (pos_l - prev_pos_l) * WHEEL_RADIUS_L; // metros recorridos rueda izq
    float dr =
        (pos_r - prev_pos_r) * WHEEL_RADIUS_R; // metros recorridos rueda der
    prev_pos_l = pos_l;
    prev_pos_r = pos_r;

    // Yaw incremental: arco diferencial / distancia entre ruedas
    odom_yaw = norm_angle(odom_yaw + (dr - dl) / (2.0f * HALF_BASE));

    /* 3. Predicción con modelo cinemático */
    ekf.predict(v, omega);

    /* 4. Corrección IMU (θ) — señal limpia gracias al fix en SensorMeasureTask
     */
    imu = imuGetState();
    if (imu->valid)
      ekf.correctIMU(imu->yaw_rad);

    /* 5. Corrección odometría (θ) — NUEVO
     *
     * Se aplica después de la IMU para que la covarianza P ya haya sido
     * reducida por la corrección IMU. Así R_ODOM compite contra una P
     * más pequeña y su influencia es proporcional a la incertidumbre real
     * restante — evita sobre-corregir cuando la IMU ya está segura.
     */
    ekf.correctOdom(odom_yaw);

    /* 6. Corrección UWB (solo cuando hay medición nueva) */
    uwb = uwbGetState();
    if (uwb->valid && uwb->timestamp_us != last_uwb_ts) {
      ekf.correctUWB(uwb->x, uwb->y);
      last_uwb_ts = uwb->timestamp_us;
      ESP_LOGD(TAG, "Correccion UWB: x=%.3f y=%.3f", uwb->x, uwb->y);
    }

    /* 7. Publicar estado */
    ekf_state.x = ekf.getX();
    ekf_state.y = ekf.getY();
    ekf_state.theta = ekf.getTheta();
    ekf_state.pos_left_rad = pos_l;
    ekf_state.pos_right_rad = pos_r;
    ekf_state.vel_left_rads = omega_l;
    ekf_state.vel_right_rads = omega_r;
    ekf_state.dir_left_fwd = !isInvertedLeft();
    ekf_state.dir_right_fwd = !isInvertedRight();
    ekf_state.timestamp_us = esp_timer_get_time();
    ekf_state.valid = true;

    ESP_LOGI(TAG,
             "Pose: x=%.3f  y=%.3f  θ_ekf=%.1f°  θ_imu=%.1f°  θ_odom=%.1f°",
             ekf_state.x, ekf_state.y, ekf_state.theta * (180.f / M_PI),
             imu->yaw_rad * (180.f / M_PI), odom_yaw * (180.f / M_PI));

    vTaskDelay(period);
  }

  vTaskDelete(NULL);
}

/* ===============================================================
 *  API pública
 * =============================================================== */
void ekfTaskStart(uint32_t period_ms) {
  xTaskCreate(ekfTask, "EKFTask", 4096, (void *)period_ms, 5, NULL);
}

const EKFState *ekfGetState(void) { return &ekf_state; }
