#include "WheelDriver.h"
#include "freertos/FreeRTOS.h"
#include "LUTStore/LUTStore.h"

WheelDriver::WheelDriver(
    MotorConfig motorConfig,
    EncoderConfig encoderConfig,
    int8_t NUM_SECTORS,
    const std::string &nvs_namespace,
    float (*lut)[2])
    : lut_(lut),
      motor_(motorConfig.PIN_A,
             motorConfig.PIN_B,
             motorConfig.CH_A,
             motorConfig.CH_B,
             motorConfig.TIMER,
             motorConfig.MODE,
             motorConfig.RESOLUTION),
      encoder_(encoderConfig.PULSE_PIN,
               encoderConfig.SECTOR_PIN,
               lut,
               encoderConfig.GLITCH_FILTER_NS),
      lut_store_(lut, NUM_SECTORS, nvs_namespace)
{
    xTaskCreatePinnedToCore(
        WheelDriver::calibrateTaskEntry,
        "CalibrateTask",
        4096,
        this,
        5,
        &calibrate_task_handle_,
        tskNO_AFFINITY);

    xTaskCreatePinnedToCore(
        WheelDriver::nvsTaskEntry,
        "NVSTask",
        4096,
        this,
        5,
        &nvs_task_handle_,
        tskNO_AFFINITY);
    encoder_.setDoneCallback(calibrate_task_handle_);
}

void WheelDriver::calibrateTaskEntry(void *arg)
{
    static_cast<WheelDriver *>(arg)->calibrateTaskLoop();
}

void WheelDriver::nvsTaskEntry(void *arg)
{
    static_cast<WheelDriver *>(arg)->nvsTaskLoop();
}

void WheelDriver::calibrateTaskLoop()
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        motor_.stop();
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!isInverted())
            setDuty(CALIBRATE_DUTY);
        else
            setDuty(-CALIBRATE_DUTY);
        encoder_.startCalibration();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        motor_.stop();
        xTaskNotifyGive(nvs_task_handle_);
    }
}

void WheelDriver::nvsTaskLoop()
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        lut_store_.saveLUT(isInverted());
    }
}

void WheelDriver::setDuty(float new_duty)
{
    motor_.setDuty(new_duty);
    if (new_duty > 0)
        encoder_.setInverted(false);
    else if (new_duty < 0)
        encoder_.setInverted(true);
    else
        encoder_.clearDT();
}

void WheelDriver::stop()
{
    motor_.stop();
    encoder_.clearDT();
    if (encoder_.isCalibrating())
        encoder_.stopCalibration();
}

void WheelDriver::loadLUT()
{
    lut_store_.loadLUT();
}

void WheelDriver::calibrate(Direction dir)
{
    if (dir == Direction::FORWARD)
        encoder_.setInverted(false);
    else
        encoder_.setInverted(true);

    xTaskNotifyGive(calibrate_task_handle_);
}

bool WheelDriver::isInverted()
{
    return encoder_.isInverted();
}

float WheelDriver::getVelocity(VelocityUnits units)
{
    return encoder_.getVelocity(units);
}