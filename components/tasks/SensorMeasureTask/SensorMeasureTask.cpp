#include "SensorMeasureTask.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include <cmath>
#include <cstddef>
#ifdef __cplusplus
extern "C" {
#endif
#include "ahrs.h"
#include "calibrate.h"
#include "common.h"
#include "mpu9250.h"
#ifdef __cplusplus
}
#endif

static const char *TAG = "SensorMeasure";

/* ===============================================================
 *  Estado publicado (escrito solo por sensorTask)
 * =============================================================== */
static IMUState imu_state;
static IMUState imu_state_copy;
static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static float yaw_offset_rad = 0.0f;

/* ===============================================================
 *  Calibración
 * =============================================================== */
static calibration_t cal = {
    .mag_offset = {.x = -62.765625, .y = 22.859375, .z = 97.199219},
    .mag_scale = {.x = 1.056077, .y = 0.976732, .z = 0.971556},
    .gyro_bias_offset = {.x = -5.176833, .y = -2.077277, .z = 0.325611},
    .accel_offset = {.x = 0.165891, .y = -0.005169, .z = 0.021756},
    .accel_scale_lo = {.x = 1.007532, .y = 0.987073, .z = 0.999830},
    .accel_scale_hi = {.x = -0.681083, .y = -1.005608, .z = -1.032582}};

/* ===============================================================
 *  Helper: grados → radianes normalizados a [-π, π]
 * =============================================================== */
static inline float deg2rad_norm(float deg) {
  float rad = deg * (M_PI / 180.0f);
  while (rad > M_PI)
    rad -= 2.0f * M_PI;
  while (rad < -M_PI)
    rad += 2.0f * M_PI;
  return rad;
}

/* ===============================================================
 *  Task
 * =============================================================== */
static void sensorTask(void *arg) {
#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  i2c_mpu9250_init(&cal);

  // Alpha para complementary filter: cuánto confías en el giroscopio
  // 0.98 = 98% gyro + 2% accel para corregir drift en pitch/roll
  static constexpr float ALPHA = 0.98f;

  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
  float yaw_deg = 0.0f;
  int64_t last_time_us = esp_timer_get_time();

  uint64_t i = 0;
  while (true) {
    vector_t va, vg, vm;
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // dt real entre muestras
    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - last_time_us) * 1e-6f;
    last_time_us = now_us;

    // Integración del giroscopio (vg ya está en deg/s tras calibración)
    static constexpr float GYRO_THRESHOLD = 0.5f; // deg/s

    if (fabsf(vg.x) < GYRO_THRESHOLD)
      vg.x = 0.0f;
    if (fabsf(vg.y) < GYRO_THRESHOLD)
      vg.y = 0.0f;
    if (fabsf(vg.z) < GYRO_THRESHOLD)
      vg.z = 0.0f;

    roll_deg += vg.x * dt;
    pitch_deg += vg.y * dt;
    yaw_deg += vg.z * dt;

    // Corrección de pitch y roll con acelerómetro
    // va está en g — atan2 da el ángulo de inclinación
    float accel_roll = atan2f(va.y, va.z) * (180.0f / M_PI);
    float accel_pitch =
        atan2f(-va.x, sqrtf(va.y * va.y + va.z * va.z)) * (180.0f / M_PI);

    // Complementary filter: fusiona gyro (rápido) con accel (sin drift)
    roll_deg = ALPHA * roll_deg + (1.0f - ALPHA) * accel_roll;
    pitch_deg = ALPHA * pitch_deg + (1.0f - ALPHA) * accel_pitch;
    // yaw no tiene corrección sin magnetómetro — driftea con el tiempo

    // Publicar estado
    float yaw = deg2rad_norm(yaw_deg) - yaw_offset_rad;
    if (yaw > M_PI)
      yaw -= 2.0f * M_PI;
    if (yaw < -M_PI)
      yaw += 2.0f * M_PI;

    IMUState tmp;
    tmp.yaw_rad = yaw;
    tmp.pitch_rad = deg2rad_norm(pitch_deg);
    tmp.roll_rad = deg2rad_norm(roll_deg);
    tmp.gyro_z_rads = DEG2RAD(vg.z);
    tmp.timestamp_us = esp_timer_get_time();
    tmp.valid = true;

    portENTER_CRITICAL(&imu_mux);
    imu_state = tmp;
    portEXIT_CRITICAL(&imu_mux);

    // if (i++ % 50 == 0) {
    //   ESP_LOGI(TAG, "yaw=%.2f°  pitch=%.2f°  roll=%.2f°", yaw_deg, pitch_deg,
    //            roll_deg);
    // }

    pause();
  }
#endif
  vTaskDelete(NULL);
}

/* ===============================================================
 *  API pública
 * =============================================================== */
void sensorMeasureTaskStart(void) {
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, NULL, 6, NULL, 0);
}

const IMUState *imuGetState(void) {
  portENTER_CRITICAL(&imu_mux);
  imu_state_copy = imu_state;
  portEXIT_CRITICAL(&imu_mux);
  return &imu_state_copy;
}
