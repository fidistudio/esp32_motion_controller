#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdint.h>
#include <string>

#include "MotorPWM/MotorPWM.h"
#include "EncoderPCNT/EncoderPCNT.h"
#include "LUTStore/LUTStore.h"

/* ================= structs ================= */

struct MotorConfig
{
    gpio_num_t PIN_A;
    gpio_num_t PIN_B;
    ledc_channel_t CH_A;
    ledc_channel_t CH_B;
    ledc_timer_t TIMER;
    ledc_mode_t MODE;
    ledc_timer_bit_t RESOLUTION;
};

struct EncoderConfig
{
    gpio_num_t PULSE_PIN;
    gpio_num_t SECTOR_PIN;
    uint32_t GLITCH_FILTER_NS = 1000;
};

/* ================= enums ================= */

enum class Direction {
    FORWARD,
    REVERSE
};

/* ================= WheelDriver ================= */

class WheelDriver
{
public:
    WheelDriver(MotorConfig motorConfig,
                EncoderConfig encoderConfig,
                int8_t NUM_SECTORS,
                const std::string &nvs_namespace,
                float (*lut)[2]);

    void setDuty(float new_duty);
    void stop();
    void loadLUT();
    void calibrate(Direction dir);
    bool isInverted();
    float getVelocity(VelocityUnits units);

private:
    float (*lut_)[2];
    Motor motor_;
    Encoder encoder_;
    LUTStore lut_store_;
    const float CALIBRATE_DUTY = 0.3;

    TaskHandle_t calibrate_task_handle_;
    static void calibrateTaskEntry(void *arg);
    void calibrateTaskLoop();
};