#include "SensorMeasureTask.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

/* ===============================================================
 *  Calibración
 * =============================================================== */
static calibration_t cal = {
    .mag_offset = {.x = 94.148438f, .y = 78.203125f, .z = 6.984375f},
    .mag_scale = {.x = 1.344917f, .y = 0.878603f, .z = 0.894223f},
    .gyro_bias_offset = {.x = -4.623848f, .y = -1.704701f, .z = 0.147797f},
    .accel_offset = {.x = 0.018234f, .y = -0.020647f, .z = -0.065179f},
    .accel_scale_lo = {.x = 0.987014f, .y = 0.983493f, .z = 0.986657f},
    .accel_scale_hi = {.x = -0.996878f, .y = -1.006448f, .z = -1.026394f}};

/* ===============================================================
 *  Transformaciones de sensor
 * =============================================================== */
static void transform_accel_gyro(vector_t *v) {
  float x = v->x, y = v->y, z = v->z;
  v->x = -x;
  v->y = -z;
  v->z = -y;
}

static void transform_mag(vector_t *v) {
  float x = v->x, y = v->y, z = v->z;
  v->x = -y;
  v->y = z;
  v->z = -x;
}

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
  ahrs_init(SAMPLE_FREQ_Hz, 0.8f);

  uint64_t i = 0;

  while (true) {
    vector_t va, vg, vm;
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z), va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

    /* --- Publicar estado IMU --- */
    {
      float yaw_deg, pitch_deg, roll_deg;
      ahrs_get_euler_in_degrees(&yaw_deg, &pitch_deg, &roll_deg);

      // Conversión a radianes y normalización [-π, π]
      imu_state.yaw_rad = deg2rad_norm(yaw_deg);
      imu_state.pitch_rad = deg2rad_norm(pitch_deg);
      imu_state.roll_rad = deg2rad_norm(roll_deg);
      imu_state.timestamp_us = esp_timer_get_time();
      imu_state.valid = true;
    }

    if (i++ % 10 == 0) {
      ESP_LOGI(TAG,
               "yaw: %.3f°  pitch: %.3f°  roll: %.3f°  "
               "(yaw_rad: %.4f)",
               imu_state.yaw_rad * (180.0f / M_PI),
               imu_state.pitch_rad * (180.0f / M_PI),
               imu_state.roll_rad * (180.0f / M_PI), imu_state.yaw_rad);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
#endif
  vTaskDelete(NULL);
}

/* ===============================================================
 *  API pública
 * =============================================================== */

void sensorMeasureTaskStart(void) {
  xTaskCreate(sensorTask, "sensorTask", 4096, NULL, 10, NULL);
}

const IMUState *imuGetState(void) { return &imu_state; }

float imuGetYawRad(void) { return imu_state.yaw_rad; }
