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
#include "portmacro.h"

#include <cmath>

static const char *TAG = "EKFTask";

/* ===============================================================
 *  Parámetros físicos del robot
 * =============================================================== */
static constexpr float WHEEL_RADIUS_L = 0.31f / 2.0f;
static constexpr float WHEEL_RADIUS_R = 0.31f / 2.0f;
static constexpr float HALF_BASE = 0.37f / 2.0f;

/* ===============================================================
 *  Parámetros del filtro
 *
 *  Guía de tuning:
 *    - Si el robot gira y el yaw tarda en seguirlo → bajar R_IMU
 *    - Si el yaw deriva con el tiempo en línea recta → bajar R_ODOM
 *    - Si el yaw salta al arrancar los motores → subir R_IMU
 *    - W_GYRO/W_ODOM: si el gyro deriva mucho → bajar W_GYRO
 * =============================================================== */
static constexpr float DT_S = 0.05f;
static constexpr float Q_XY = 0.001f;
static constexpr float Q_THETA =
    0.001f; // subido: sin mag el modelo es menos seguro
static constexpr float R_UWB = 0.04f;
static constexpr float R_IMU = 0.05f; // subido: Madgwick sin mag deriva más
static constexpr float R_ODOM =
    0.02f; // bajado: odometría es referencia principal

// Pesos para fusión omega en predicción
static constexpr float W_GYRO =
    0.7f; // giroscopio: rápido, sin drift a corto plazo
static constexpr float W_ODOM = 0.3f; // odometría: sin drift a largo plazo

/* ===============================================================
 *  Estado publicado
 * =============================================================== */
static EKFState ekf_state;
static EKFState ekf_state_copy;
static portMUX_TYPE ekf_mux = portMUX_INITIALIZER_UNLOCKED;

/* ===============================================================
 *  Helper: normalización angular [-π, π]
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

  EKF ekf(DT_S, Q_XY, Q_THETA, R_UWB, R_IMU, R_ODOM);

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

  int64_t last_uwb_ts = uwb->timestamp_us;

  /* ---- Acumuladores de odometría ---- */
  float odom_yaw = theta0;
  float prev_pos_l = getPositionLeft();
  float prev_pos_r = getPositionRight();

  /* ---- Loop principal ---- */
  while (true) {

    /* 1. Snapshot de ruedas */
    float omega_l, omega_r, pos_l, pos_r;
    getWheelSnapshot(&omega_l, &omega_r, &pos_l, &pos_r);

    /* 2. Leer IMU — necesitamos gyro_z para la predicción */
    imu = imuGetState();

    /* 3. Velocidad lineal y angular por odometría */
    float v = (WHEEL_RADIUS_R * omega_r + WHEEL_RADIUS_L * omega_l) / 2.0f;
    float omega_odom = (WHEEL_RADIUS_R * omega_r - WHEEL_RADIUS_L * omega_l) /
                       (2.0f * HALF_BASE);

    /* 4. Fusionar omega_odom y omega_gyro para la predicción */
    float omega_gyro = imu->valid ? imu->gyro_z_rads : omega_odom;
    float omega_pred = W_GYRO * omega_gyro + W_ODOM * omega_odom;

    /* 5. Acumulación de yaw odométrico por posición de encoder */
    float dl = (pos_l - prev_pos_l) * WHEEL_RADIUS_L;
    float dr = (pos_r - prev_pos_r) * WHEEL_RADIUS_R;
    prev_pos_l = pos_l;
    prev_pos_r = pos_r;
    odom_yaw = norm_angle(odom_yaw + (dr - dl) / (2.0f * HALF_BASE));

    /* 6. Predicción con omega fusionado */
    ekf.predict(v, omega_pred);

    /* 7. Corrección IMU — yaw Madgwick sin magnetómetro */
    if (imu->valid)
      ekf.correctIMU(imu->yaw_rad);

    /* 8. Corrección odometría — se aplica después de IMU para que P
     *    ya haya sido reducida y R_ODOM compita contra incertidumbre real */
    ekf.correctOdom(odom_yaw);

    /* 9. Corrección UWB — solo cuando hay medición nueva */
    uwb = uwbGetState();
    if (uwb->valid && uwb->timestamp_us != last_uwb_ts) {
      ekf.correctUWB(uwb->x, uwb->y);
      last_uwb_ts = uwb->timestamp_us;
      ESP_LOGD(TAG, "Correccion UWB: x=%.3f y=%.3f", uwb->x, uwb->y);
    }

    /* 10. Publicar estado con mutex */
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

    ESP_LOGI(TAG,
             "Pose: x=%.3f  y=%.3f  θ_ekf=%.1f°  θ_imu=%.1f°  θ_odom=%.1f°",
             tmp.theta * (180.f / M_PI), imu->yaw_rad * (180.f / M_PI),
             odom_yaw * (180.f / M_PI));

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
