#pragma once
#include "EncoderPCNT/EncoderPCNT.h"
#include "LUTStore/LUTStore.h"
#include "MotorPWM/MotorPWM.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include <stdint.h>
#include <string>

struct MotorConfig {
  gpio_num_t PIN_A;
  gpio_num_t PIN_B;
  ledc_channel_t CH_A;
  ledc_channel_t CH_B;
  ledc_timer_t TIMER;
  ledc_mode_t MODE;
  ledc_timer_bit_t RESOLUTION;
};
struct EncoderConfig {
  gpio_num_t PULSE_PIN;
  gpio_num_t SECTOR_PIN;
  uint32_t GLITCH_FILTER_NS = 1000;
};

enum class Direction { FORWARD, REVERSE };

class WheelDriver {
public:
  WheelDriver(MotorConfig motorConfig, EncoderConfig encoderConfig,
              int8_t NUM_SECTORS, const std::string &nvs_namespace,
              float (*lut)[2], BaseType_t core_id = tskNO_AFFINITY);
  void setDuty(float new_duty);
  void stop();
  void loadLUT();
  void calibrate(Direction dir);
  bool isInverted();
  float getVelocity(VelocityUnits units);
  float getPosition();
  void resetPosition();

private:
  float (*lut_)[2];
  Motor motor_;
  Encoder encoder_;
  LUTStore lut_store_;
  const float CALIBRATE_DUTY = 0.3;
  TaskHandle_t calibrate_task_handle_;
  TaskHandle_t nvs_task_handle_; // ← added
  static void calibrateTaskEntry(void *arg);
  void calibrateTaskLoop();
  static void nvsTaskEntry(void *arg); // ← added
  void nvsTaskLoop();                  // ← added
};
