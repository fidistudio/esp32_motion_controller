#include "SensorMeasureTask.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

// Aquí va la configuración de calibración, como en tu ejemplo
static calibration_t cal = {
    // Magnetometer
    .mag_offset = {.x = 94.148438, .y = 78.203125, .z = 6.984375},
    .mag_scale = {.x = 1.344917, .y = 0.878603, .z = 0.894223},

    // Gyroscope
    .gyro_bias_offset = {.x = -4.623848, .y = -1.704701, .z = 0.147797},

    // Accelerometer
    .accel_offset = {.x = 0.018234, .y = -0.020647, .z = -0.065179},
    .accel_scale_lo = {.x = 0.987014, .y = 0.983493, .z = 0.986657},
    .accel_scale_hi = {.x = -0.996878, .y = -1.006448, .z = -1.026394}};

// Transformaciones de sensor
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

// Task que corre el loop de sensores
static void sensorTask(void *arg) {
#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  i2c_mpu9250_init(&cal);
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);

  uint64_t i = 0;
  while (true) {
    vector_t va, vg, vm;

    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z), va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

    if (i++ % 10 == 0) {
      float heading, pitch, roll, temp;
      ESP_ERROR_CHECK(get_temperature_celsius(&temp));
      ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
      ESP_LOGI(TAG, "heading: %.3f°, pitch: %.3f°, roll: %.3f°, Temp %.2f°C",
               heading, pitch, roll, temp);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
#endif

  vTaskDelete(NULL);
}

TaskHandle_t sensorMeasureTaskStart(void) {
  TaskHandle_t handle = NULL;
  xTaskCreate(sensorTask, "sensorTask", 4096, NULL, 10, &handle);
  return handle;
}
