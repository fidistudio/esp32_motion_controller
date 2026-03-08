#include "EKFTask.h"

#include "EKF/EKF.h"
#include "SensorMeasureTask/SensorMeasureTask.h"
#include "UWBMeasureTask/UWBMeasureTask.h"
#include "data.h" // getVelocityLeft/Right, getPositionLeft/Right, isInvertedLeft/Right

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>

static const char *TAG = "EKFTask";

/* ===============================================================
 *  Parámetros físicos del robot
 * =============================================================== */
static constexpr float WHEEL_RADIUS_L = 0.31f / 2.0f; // metros
static constexpr float WHEEL_RADIUS_R = 0.31f / 2.0f;
static constexpr float HALF_BASE = 0.37f / 2.0f; // b

/* ===============================================================
 *  Parámetros del filtro
 *
 *  q_xy     : ruido de proceso en posición [m²]
 *             Sube si hay resbalamiento o imprecisión en dt
 *  q_theta  : ruido de proceso en orientación [rad²]
 *             Sube si la IMU deriva rápido
 *  r_uwb    : ruido de medición UWB [m²]  (~20 cm → 0.04)
 *             Baja si las anclas son precisas y el entorno limpio
 *  r_imu    : ruido de medición IMU/AHRS para θ [rad²]
 *             Baja si confías en tu filtro AHRS
 * =============================================================== */
static constexpr float DT_S = 0.05f;
static constexpr float Q_XY = 0.001f;
static constexpr float Q_THETA = 0.0005f;
static constexpr float R_UWB = 0.04f;
static constexpr float R_IMU = 0.001f;

/* ===============================================================
 *  Estado publicado
 * =============================================================== */
static EKFState ekf_state;

/* ===============================================================
 *  Loop principal
 * =============================================================== */
static void ekfTask(void *arg) {
  const TickType_t period = pdMS_TO_TICKS(reinterpret_cast<uint32_t>(arg));

  EKF ekf(DT_S, Q_XY, Q_THETA, R_UWB, R_IMU);

  int64_t last_uwb_ts = 0;

  ESP_LOGI(TAG, "EKFTask iniciada a %lu ms", (uint32_t)(DT_S * 1000));

  while (true) {
    /* ---- 1. Odometría diferencial → v, ω ---- */
    float omega_l = getVelocityLeft(VelocityUnits::RAD_S);
    float omega_r = getVelocityRight(VelocityUnits::RAD_S);

    float v = (WHEEL_RADIUS_R * omega_r + WHEEL_RADIUS_L * omega_l) / 2.0f;
    float omega = (WHEEL_RADIUS_R * omega_r - WHEEL_RADIUS_L * omega_l) /
                  (2.0f * HALF_BASE);

    /* ---- 2. Predicción EKF ---- */
    ekf.predict(v, omega);

    /* ---- 3. Corrección IMU (θ) ---- */
    const IMUState *imu = imuGetState();
    if (imu->valid)
      ekf.correctIMU(imu->yaw_rad);

    /* ---- 4. Corrección UWB (solo cuando hay medición nueva) ---- */
    const UWBState *uwb = uwbGetState();
    if (uwb->valid && uwb->timestamp_us != last_uwb_ts) {
      ekf.correctUWB(uwb->x, uwb->y);
      last_uwb_ts = uwb->timestamp_us;
      ESP_LOGD(TAG, "Correccion UWB: x=%.3f y=%.3f", uwb->x, uwb->y);
    }

    /* ---- 5. Publicar estado ---- */
    // Pose EKF
    ekf_state.x = ekf.getX();
    ekf_state.y = ekf.getY();
    ekf_state.theta = ekf.getTheta();

    // Rueda izquierda
    ekf_state.pos_left_rad = getPositionLeft();
    ekf_state.vel_left_rads = omega_l;
    ekf_state.dir_left_fwd = !isInvertedLeft();

    // Rueda derecha
    ekf_state.pos_right_rad = getPositionRight();
    ekf_state.vel_right_rads = omega_r;
    ekf_state.dir_right_fwd = !isInvertedRight();

    ekf_state.timestamp_us = esp_timer_get_time();
    ekf_state.valid = true;

    ESP_LOGD(TAG,
             "Pose x=%.3f y=%.3f θ=%.3f | "
             "L pos=%.2f vel=%.2f | R pos=%.2f vel=%.2f",
             ekf_state.x, ekf_state.y, ekf_state.theta, ekf_state.pos_left_rad,
             ekf_state.vel_left_rads, ekf_state.pos_right_rad,
             ekf_state.vel_right_rads);

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
