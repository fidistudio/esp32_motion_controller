#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

// ---------------- Encoder ----------------
constexpr gpio_num_t ENCODER_GPIO = GPIO_NUM_23;
constexpr gpio_num_t SECTOR_GPIO = GPIO_NUM_22;

// ---------------- Motor 1 ----------------
constexpr gpio_num_t MOTOR1_IN1_PIN = GPIO_NUM_25;
constexpr gpio_num_t MOTOR1_IN2_PIN = GPIO_NUM_26;

constexpr ledc_channel_t MOTOR1_CH_A = LEDC_CHANNEL_0;
constexpr ledc_channel_t MOTOR1_CH_B = LEDC_CHANNEL_1;
