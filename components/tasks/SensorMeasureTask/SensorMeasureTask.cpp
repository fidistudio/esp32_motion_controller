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
  ahrs_init(SAMPLE_FREQ_Hz, 0.05f); // <-- beta igual que main.c

  /* --- Inicializar cuaternión con orientación real del sensor --- */
  {
    vector_t va, vg, vm;
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));
    transform_accel_gyro(&va);
    transform_mag(&vm);
    ahrs_init_from_sensors(va.x, va.y, va.z, vm.x, vm.y, vm.z);
  }

  uint64_t i = 0;
  while (true) {
    vector_t va, vg, vm;

    /* --- Leer acelerómetro, giroscopio y magnetómetro --- */
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    /* --- Transformar ejes --- */
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    /* --- Algoritmo AHRS con mag (igual que main.c) --- */
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z), va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

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
      tmp.timestamp_us = esp_timer_get_time();
      tmp.valid = true;

      portENTER_CRITICAL(&imu_mux);
      imu_state = tmp;
      portEXIT_CRITICAL(&imu_mux);
    }

    /* --- Log cada 10 muestras (igual que main.c) --- */
    if (i++ % 10 == 0) {
      float temp;
      ESP_ERROR_CHECK(get_temperature_celsius(&temp));

      ESP_LOGI(TAG,
               "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C",
               imu_state.yaw_rad * (180.0f / M_PI),
               imu_state.pitch_rad * (180.0f / M_PI),
               imu_state.roll_rad * (180.0f / M_PI), temp);

      vTaskDelay(0); // mantener el WDT contento
    }

    pause(); // <-- igual que main.c, reemplaza el delay fijo de 10 ms
  }
#endif
  vTaskDelete(NULL);
}

/* ===============================================================
 *  API pública
 * =============================================================== */
void sensorMeasureTaskStart(void) {
  xTaskCreate(sensorTask, "sensorTask", 4096, NULL, 6, NULL);
}

const IMUState *imuGetState(void) {
  portENTER_CRITICAL(&imu_mux);
  imu_state_copy = imu_state;
  portEXIT_CRITICAL(&imu_mux);
  return &imu_state_copy;
}

void imuResetYawOffset(void) {
  yaw_offset_rad = imu_state.yaw_rad + yaw_offset_rad; // acumular offsets
  ESP_LOGI(TAG, "Offset puesto en: %.3f rad (%.3f°)", yaw_offset_rad,
           yaw_offset_rad * (180.0f / M_PI));
}
