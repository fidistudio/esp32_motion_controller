#pragma once
#include "EncoderPCNT/EncoderPCNT.h"
#include "WheelDriver/WheelDriver.h"
#include "freertos/FreeRTOS.h"

void dataInit(void);
void setDuty(float new_duty_left, float new_duty_right);
void setTargetSpeed(float linear, float angular);
void stop(void);
void calibrate(Direction dir);
float getVelocityLeft(VelocityUnits units);
float getVelocityRight(VelocityUnits units);
float getPositionLeft(void);
float getPositionRight(void);
void getWheelSnapshot(float *vel_l, float *vel_r, float *pos_l, float *pos_r);
void resetPositions(void);
bool isInvertedLeft(void);
bool isInvertedRight(void);
void controlUpdate(void);
void printLUT(void);

bool is_timer_enabled(void);
void set_timer_state(bool state);
