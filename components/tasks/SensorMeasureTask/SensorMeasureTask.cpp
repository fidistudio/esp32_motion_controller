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
    .mag_offset = {.x = -56.730469, .y = 8.421875, .z = 120.480469},
    .mag_scale = {.x = 1.040260, .y = 0.913183, .z = 1.059735},
    .gyro_bias_offset = {.x = -5.275731, .y = -2.099989, .z = 0.294089},
    .accel_offset = {.x = 0.035765, .y = -0.011386, .z = -0.026014},
    .accel_scale_lo = {.x = 1.005573, .y = 0.988421, .z = 0.996858},
    .accel_scale_hi = {.x = -0.991350, .y = -1.003926, .z = -1.034768}};

/* ===============================================================
 *  Transformaciones de ejes
 * =============================================================== */
static void transform_accel_gyro(vector_t *v) {
  float x = v->x, y = v->y, z = v->z;
  v->x = -x;
  v->y = -z;
  v->z = -y;
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

  /* --- Inicializar cuaternión solo con acelerómetro (sin mag) ---
   * Beta alto (0.5) para convergencia rápida, luego restaurar a 0.05
   * El yaw inicial será 0 — sin magnetómetro no hay referencia de norte
   */
  {
    vector_t va, vg, vm;
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));
    transform_accel_gyro(&va);

    ahrs_init(SAMPLE_FREQ_Hz, 0.5f);
    for (int k = 0; k < 200; k++) {
      ahrs_update(0.0f, 0.0f, 0.0f, va.x, va.y, va.z, 0.0f, 0.0f, 0.0f);
    }
    ahrs_init(SAMPLE_FREQ_Hz, 0.05f);
  }

  uint64_t i = 0;
  while (true) {
    vector_t va, vg;

    /* --- Leer acelerómetro y giroscopio (mag ignorado) --- */
    {
      vector_t vm_unused;
      ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm_unused));
    }

    /* --- Transformar ejes --- */
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);

    /* --- Madgwick IMU — sin magnetómetro --- */
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z), va.x, va.y, va.z,
                0.0f, 0.0f, 0.0f);

    /* --- Publicar estado IMU --- */
    {
      float yaw_deg, pitch_deg, roll_deg;
      ahrs_get_euler_in_degrees(&yaw_deg, &pitch_deg, &roll_deg);

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

      /* --- Log cada 10 muestras --- */
      if (i++ % 10 == 0) {
        float temp;
        ESP_ERROR_CHECK(get_temperature_celsius(&temp));
        ESP_LOGI(TAG,
                 "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C",
                 tmp.yaw_rad * (180.0f / M_PI), tmp.pitch_rad * (180.0f / M_PI),
                 tmp.roll_rad * (180.0f / M_PI), temp);
        vTaskDelay(0);
      }
    }

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

void imuResetYawOffset(void) {
  const IMUState *s = imuGetState();
  yaw_offset_rad = s->yaw_rad + yaw_offset_rad;
  ESP_LOGI(TAG, "Offset puesto en: %.3f rad (%.3f°)", yaw_offset_rad,
           yaw_offset_rad * (180.0f / M_PI));
}
