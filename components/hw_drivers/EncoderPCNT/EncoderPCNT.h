#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#include <stdint.h>

/* ================= structs ================= */

struct EncoderSample
{
    int8_t sector;
    int64_t dt_us;
};

enum class VelocityUnits : uint8_t
{
    RPM,
    RAD_S
};

/* ================= PCNT ================= */

class PCNTPulse
{
public:
    using isr_cb_t = bool (*)(pcnt_unit_handle_t,
                              const pcnt_watch_event_data_t *,
                              void *);

    PCNTPulse(gpio_num_t pulse_gpio,
              isr_cb_t callback,
              void *callback_arg,
              uint32_t glitch_filter_ns = 100);

    void clearCount();

private:
    pcnt_unit_handle_t unit_;
    pcnt_channel_handle_t channel_;
};

/* ================= EncoderPulseSource ================= */

class EncoderPulseSource
{
public:
    EncoderPulseSource(
        gpio_num_t pulse_gpio,
        gpio_num_t sector0_gpio,
        QueueHandle_t queue,
        int8_t num_sectors = 15,
        uint32_t glitch_filter_ns = 100);

    void setInverted(bool inverted);

private:
    static bool IRAM_ATTR onPulse(
        pcnt_unit_handle_t,
        const pcnt_watch_event_data_t *,
        void *arg);

    static bool IRAM_ATTR onSector0(
        pcnt_unit_handle_t,
        const pcnt_watch_event_data_t *,
        void *arg);

    QueueHandle_t queue_;

    volatile int8_t sector_;
    volatile bool inverted_;
    int64_t last_time_us_;
    const int8_t NUM_SECTORS_;

    PCNTPulse pulse_pcnt_;
    PCNTPulse sector0_pcnt_;
};

/* ================= Encoder ================= */

class Encoder
{
public:
    Encoder(gpio_num_t pulse_pin,
            gpio_num_t sector_pin,
            float (*lut)[2],
            uint32_t glitch_filter_ns = 1000);

    void startCalibration();
    void stopCalibration();
    bool isCalibrating();
    void setInverted(bool inverted);
    bool isInverted();
    float getVelocity(VelocityUnits units);
    void setDoneCallback(TaskHandle_t task);

private:
    static void taskEntry(void *arg);
    void taskLoop();

    void calibrateStep(const EncoderSample &sample);

    // Calibration & config
    float (*lut_)[2];
    static constexpr int8_t NUM_SECTORS_ = 15;
    static constexpr int8_t MAX_CALIBRATION_REVS_ = 15;
    bool calibrating_;
    bool calibration_requested_;
    bool inverted_;

    // State
    int8_t sector_;
    int64_t dt_;
    int8_t revs_;
    int64_t sector_time[NUM_SECTORS_][MAX_CALIBRATION_REVS_];

    // Queues
    QueueHandle_t input_queue_;

    // Hardware / drivers
    EncoderPulseSource pulse_source_;

    // Task
    TaskHandle_t task_handle_;
    TaskHandle_t done_task_;

    static portMUX_TYPE mux_;
};
