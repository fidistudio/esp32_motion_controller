#include "data.h"
#include "EncoderPCNT/EncoderPCNT.h"
#include "PIDController/PIDController.h"
#include "WheelDriver.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/gpio_num.h"
#include <string>

/* ================= Configuración ================= */

static const ledc_timer_t MOTOR_TIMER = LEDC_TIMER_0;
static const ledc_mode_t MOTOR_MODE = LEDC_HIGH_SPEED_MODE;
static const ledc_timer_bit_t MOTOR_RESOLUTION = LEDC_TIMER_10_BIT;
static const uint32_t MOTOR_PWM_FREQ = 20000; // 20 kHz

static const int8_t NUM_SECTORS = 15;
static constexpr float WHEEL_RADIUS_LEFT = 0.31f / 2.0f; // meters
static constexpr float WHEEL_RADIUS_RIGHT = 0.31 / 2.0f; // meters
static constexpr float WHEEL_BASE = 0.37f;               // meters

static float target_speed_left_ = 0;
static float target_speed_right_ = 0;

static MotorConfig motor_left_cfg = {
    GPIO_NUM_26, GPIO_NUM_25, LEDC_CHANNEL_0,  LEDC_CHANNEL_1,
    MOTOR_TIMER, MOTOR_MODE,  MOTOR_RESOLUTION};

static MotorConfig motor_right_cfg = {
    GPIO_NUM_27, GPIO_NUM_14, LEDC_CHANNEL_2,  LEDC_CHANNEL_3,
    MOTOR_TIMER, MOTOR_MODE,  MOTOR_RESOLUTION};

static EncoderConfig encoder_left_cfg = {GPIO_NUM_23, GPIO_NUM_22};
static EncoderConfig encoder_right_cfg = {GPIO_NUM_19, GPIO_NUM_21};

float lut_left[NUM_SECTORS][2];
float lut_right[NUM_SECTORS][2];

static PIDGains pid_gains_left = {0.0, 0.257737, 0.0};
static PIDGains pid_gains_right = {0.0, 0.263106, 0.0};

static PIDTiming pid_timing = {0.01, 1e-6};

/* ================= NVS ================= */

static const std::string nvs_namespace_left = "wheel_left";
static const std::string nvs_namespace_right = "wheel_right";

/* ================= Timer Interrupt ================= */

static bool timer_enabled;

/* ================= Instancias ================= */

static WheelDriver *wheel_right = nullptr;
static WheelDriver *wheel_left = nullptr;

static PIDController *controller_right = nullptr;
static PIDController *controller_left = nullptr;

/* ================= Data ================= */

static const char *TAG = "Data";

void dataInit(void) {
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode = MOTOR_MODE;
  timer_conf.duty_resolution = MOTOR_RESOLUTION;
  timer_conf.timer_num = MOTOR_TIMER;
  timer_conf.freq_hz = MOTOR_PWM_FREQ;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;

  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < NUM_SECTORS; i++) {
      lut_left[i][j] = 100.0f;
      lut_right[i][j] = 100.0f;
    }
  }

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  controller_left = new PIDController(pid_gains_left, pid_timing);
  wheel_left = new WheelDriver(motor_left_cfg, encoder_left_cfg, NUM_SECTORS,
                               nvs_namespace_left, lut_left);
  wheel_left->loadLUT();

  controller_right = new PIDController(pid_gains_right, pid_timing);
  wheel_right = new WheelDriver(motor_right_cfg, encoder_right_cfg, NUM_SECTORS,
                                nvs_namespace_right, lut_right);
  wheel_right->loadLUT();

  ESP_LOGI(TAG, "Data inicializado");
}

/* ================= WheelDriver ================= */

void setDuty(float new_duty_left, float new_duty_right) {
  wheel_left->setDuty(new_duty_left);
  wheel_right->setDuty(new_duty_right);
}

void setTargetSpeed(float linear, float angular) {
  float omega_l = (linear - 0.5f * WHEEL_BASE * angular) / WHEEL_RADIUS_LEFT;
  float omega_r = (linear + 0.5f * WHEEL_BASE * angular) / WHEEL_RADIUS_RIGHT;
  target_speed_right_ = omega_r;
  target_speed_left_ = omega_l;
}

void stop(void) {
  wheel_left->stop();
  wheel_right->stop();
}

void calibrate(Direction dir) {
  wheel_left->calibrate(dir);
  wheel_right->calibrate(dir);
}

float getVelocityLeft(VelocityUnits units) {
  return wheel_left->getVelocity(units);
}

float getVelocityRight(VelocityUnits units) {
  return wheel_right->getVelocity(units);
}

void controlUpdate(void) {
  float error_left =
      target_speed_left_ - wheel_left->getVelocity(VelocityUnits::RAD_S);

  float duty_left = controller_left->update(error_left);

  float error_right =
      target_speed_right_ - wheel_right->getVelocity(VelocityUnits::RAD_S);
  float duty_right = controller_right->update(error_right);

  wheel_left->setDuty(duty_left);
  wheel_right->setDuty(duty_right);
  ESP_LOGI(TAG, "Error Left: %f Error Right: %f", error_left, error_right);
}

void printLUT(void) {
  ESP_LOGI(TAG, "LUT_LEFT\tFORWARD\t\tREVERSE");
  for (int i = 0; i < NUM_SECTORS; i++)
    ESP_LOGI(TAG, "[%d]\t%f\t%f", i, lut_left[i][0], lut_left[i][1]);

  ESP_LOGI(TAG, "LUT_RIGHT\tFORWARD\t\tREVERSE");
  for (int i = 0; i < NUM_SECTORS; i++)
    ESP_LOGI(TAG, "[%d]\t%f\t%f", i, lut_right[i][0], lut_right[i][1]);
}

/* ================= TimerNotify ================= */

bool is_timer_enabled(void) { return timer_enabled; }

void set_timer_state(bool state) { timer_enabled = state; }
