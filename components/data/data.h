#pragma once
#include "WheelDriver/WheelDriver.h"
#include "EncoderPCNT/EncoderPCNT.h"
#include "freertos/FreeRTOS.h"

void dataInit(void);
void setDuty(float new_duty);
void setTargetSpeed(float targetSpeed);
float getVelocity(VelocityUnits units);
void stop(void);
void calibrate(Direction dir);
void controlUpdate(void);
void printLUT(void);

bool is_timer_enabled(void);
void set_timer_state(bool state);