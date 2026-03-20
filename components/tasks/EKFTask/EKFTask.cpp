#include "EKFTask.h"
#include "EKF/EKF.h"
#include "SensorMeasureTask/SensorMeasureTask.h"
#include "UWBMeasureTask/UWBMeasureTask.h"
#include "data.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include <cmath>

static const char *TAG = "EKFTask";

/* ===============================================================
 *  Parámetros físicos
 * =============================================================== */
static constexpr float WHEEL_RADIUS = 0.31f / 2.0f;
static constexpr float HALF_BASE = 0.37f / 2.0f;

/* ===============================================================
 *  Ruido del filtro
 *
 *  q_xy, q_theta  — ruido de proceso: cuánto confías en el modelo
 *                   cinemático. Súbelos si las ruedas tienen mucho slip.
 *  r_uwb          — ruido UWB en metros². 0.04 = ~20cm de std dev.
 *  r_imu          — ruido del yaw del gyro. Bajo = confía más en gyro.
 *  (r_odom ya no se usa — odometría solo va en predict)
 * =============================================================== */
static constexpr float DT_S = 0.05f;
static constexpr float Q_XY = 0.001f;
static constexpr float Q_THETA = 0.005f;
static constexpr float R_UWB = 0.04f;
static constexpr float R_IMU = 0.05f;

/* ===============================================================
 *  Estado publicado
 * =============================================================== */
static EKFState ekf_state;
static EKFState ekf_state_copy;
static portMUX_TYPE ekf_mux = portMUX_INITIALIZER_UNLOCKED;

/* ===============================================================
 *  Loop principal
 * =============================================================== */
static void ekfTask(void *arg) {
  const TickType_t period = pdMS_TO_TICKS(reinterpret_cast<uint32_t>(arg));

  // r_odom se pasa pero correctOdom nunca se llama — no afecta nada
  EKF ekf(DT_S, Q_XY, Q_THETA, R_UWB, R_IMU, 1.0f);

  /* ---- Esperar primera medición UWB válida ---- */
  ESP_LOGI(TAG, "Esperando primera medicion UWB...");
  const UWBState *uwb = uwbGetState();
  while (!uwb->valid) {
    vTaskDelay(pdMS_TO_TICKS(100));
    uwb = uwbGetState();
  }

  /* ---- Inicializar EKF con posición UWB y yaw IMU ---- */
  const IMUState *imu = imuGetState();
  float theta0 = imu->valid ? imu->yaw_rad : 0.0f;
  ekf.reset(uwb->x, uwb->y, theta0);
  ESP_LOGI(TAG, "EKF inicializado en x=%.3f y=%.3f theta=%.3f rad", uwb->x,
           uwb->y, theta0);

  /* ---- Variables de estado para UWB heading ---- */
  int64_t last_uwb_ts = uwb->timestamp_us;

  /* ---- Loop principal ---- */
  while (true) {

    /* 1. Odometría de ruedas → predict
     *    omega_l, omega_r en rad/s de la rueda
     *    v     = velocidad lineal del robot [m/s]
     *    omega = velocidad angular del robot [rad/s] */
    float omega_l, omega_r, pos_l, pos_r;
    getWheelSnapshot(&omega_l, &omega_r, &pos_l, &pos_r);

    float v = (WHEEL_RADIUS * omega_r + WHEEL_RADIUS * omega_l) / 2.0f;
    float omega =
        (WHEEL_RADIUS * omega_r - WHEEL_RADIUS * omega_l) / (2.0f * HALF_BASE);

    ekf.predict(v, omega);

    /* 2. IMU → correctIMU
     *    Corrige el theta que predict acumuló con el gyro integrado.
     *    Sin magnetómetro deriva, pero el UWB heading lo ancora
     *    cuando hay traslación. */
    imu = imuGetState();
    if (imu->valid)
      ekf.correctIMU(imu->yaw_rad);

    /* 3. UWB → correctUWB + yaw heading
     *    Corrige la predicción de la odometría con base
     *    en las mediciones a las antenas */
    uwb = uwbGetState();
    if (uwb->valid && uwb->timestamp_us != last_uwb_ts) {
      ekf.correctUWB(uwb->x, uwb->y);
      last_uwb_ts = uwb->timestamp_us;
    }

    /* 4. Publicar estado */
    EKFState tmp;
    tmp.x = ekf.getX();
    tmp.y = ekf.getY();
    tmp.theta = ekf.getTheta();
    tmp.pos_left_rad = pos_l;
    tmp.pos_right_rad = pos_r;
    tmp.vel_left_rads = omega_l;
    tmp.vel_right_rads = omega_r;
    tmp.dir_left_fwd = !isInvertedLeft();
    tmp.dir_right_fwd = !isInvertedRight();
    tmp.timestamp_us = esp_timer_get_time();
    tmp.valid = true;

    portENTER_CRITICAL(&ekf_mux);
    ekf_state = tmp;
    portEXIT_CRITICAL(&ekf_mux);

    ESP_LOGI(TAG, "x=%.3f y=%.3f theta=%.1f°", tmp.x, tmp.y,
             tmp.theta * (180.f / M_PI));

    vTaskDelay(period);
  }

  vTaskDelete(NULL);
}

/* ===============================================================
 *  API pública
 * =============================================================== */
void ekfTaskStart(uint32_t period_ms) {
  xTaskCreatePinnedToCore(ekfTask, "EKFTask", 4096, (void *)period_ms, 5, NULL,
                          0);
}

const EKFState *ekfGetState(void) {
  portENTER_CRITICAL(&ekf_mux);
  ekf_state_copy = ekf_state;
  portEXIT_CRITICAL(&ekf_mux);
  return &ekf_state_copy;
}
