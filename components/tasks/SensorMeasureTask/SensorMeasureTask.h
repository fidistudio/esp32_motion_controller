#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Estado IMU/AHRS publicado por SensorMeasureTask.
 */
struct IMUState {
  float yaw_rad;        ///< Yaw en radianes [-π, π], antihorario positivo
  float pitch_rad;      ///< Pitch en radianes
  float roll_rad;       ///< Roll en radianes
  float gyro_z_rads;    ///< Angular Vel Z
  int64_t timestamp_us; ///< Timestamp esp_timer_get_time()
  bool valid;           ///< true una vez que el AHRS convergió
};

/** Inicia la task de sensores */
void sensorMeasureTaskStart(void);

/** Retorna puntero al estado IMU más reciente */
const IMUState *imuGetState(void);
