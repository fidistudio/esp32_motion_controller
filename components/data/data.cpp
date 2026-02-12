#include "data.h"
#include "PIDController/PIDController.h"
#include "EncoderPCNT/EncoderPCNT.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string>

/* ================= Configuración ================= */

static const ledc_timer_t MOTOR_TIMER = LEDC_TIMER_0;
static const ledc_mode_t MOTOR_MODE = LEDC_HIGH_SPEED_MODE;
static const ledc_timer_bit_t MOTOR_RESOLUTION = LEDC_TIMER_10_BIT;
static const uint32_t MOTOR_PWM_FREQ = 20000; // 20 kHz

static const int8_t NUM_SECTORS = 15;

static float target_speed_ = 0;

static MotorConfig motor1_cfg = {
    GPIO_NUM_25,
    GPIO_NUM_26,
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    MOTOR_TIMER,
    MOTOR_MODE,
    MOTOR_RESOLUTION};

static EncoderConfig encoder1_cfg = {
    GPIO_NUM_22,
    GPIO_NUM_23};

float lut1[NUM_SECTORS][2];

/* static PIDGains pid_gains_right = {
    0.057898,
    0.773253,
    0.000000}; */

static PIDGains pid_gains_left = {
    0.056569,
    0.757286,
    0.000000};

static PIDTiming pid_timing = {
    0.01,
    0.001};

/* ================= NVS ================= */

static const std::string nvs_namespace = "wheel1";

/* ================= Timer Interrupt ================= */

static bool timer_enabled;

/* ================= Instancias ================= */

static WheelDriver *wheel1 = nullptr;
static PIDController *controller1 = nullptr;

/* ================= Data ================= */

static const char *TAG = "Data";

void dataInit(void)
{
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = MOTOR_MODE;
    timer_conf.duty_resolution = MOTOR_RESOLUTION;
    timer_conf.timer_num = MOTOR_TIMER;
    timer_conf.freq_hz = MOTOR_PWM_FREQ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    for (int j = 0; j < 2; j++)
        for (int i = 0; i < NUM_SECTORS; i++)
            lut1[i][j] = 100.0f;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    controller1 = new PIDController(pid_gains_left, pid_timing);
    wheel1 = new WheelDriver(motor1_cfg, encoder1_cfg, NUM_SECTORS, "wheel1 ", lut1);
    wheel1->loadLUT();

    ESP_LOGI(TAG, "Data inicializado");
}

/* ================= WheelDriver ================= */

void setDuty(float new_duty)
{
    wheel1->setDuty(new_duty);
}

void setTargetSpeed(float target_speed)
{
    target_speed_ = target_speed;
}

void stop(void)
{
    wheel1->stop();
}

void calibrate(Direction dir)
{
    wheel1->calibrate(dir);
}

float getVelocity(VelocityUnits units)
{
    return wheel1->getVelocity(units);
}

void controlUpdate(void)
{
    float duty = controller1->update(target_speed_ - wheel1->getVelocity(VelocityUnits::RAD_S));
    wheel1->setDuty(duty);
    ESP_LOGI(TAG, "Duty: %f", duty);
}

void printLUT(void)
{
    ESP_LOGI(TAG, "LUT1\tFORWARD\t\tREVERSE");
    for (int i = 0; i < NUM_SECTORS; i++)
        ESP_LOGI(TAG, "[%d]\t%f\t%f", i, lut1[i][0], lut1[i][1]);
}

/* ================= TimerNotify ================= */

bool is_timer_enabled(void)
{
    return timer_enabled;
}

void set_timer_state(bool state)
{
    timer_enabled = state;
}