#include "ControlTask/ControlTask.h"
#include "EKFTask/EKFTask.h"
#include "SensorMeasureTask/SensorMeasureTask.h"
#include "TimerNotifyTask/TimerNotifyTask.h"
#include "UARTCommandTask/UARTCommandTask.h"
#include "UWBMeasureTask/UWBMeasureTask.h"
#include "data.h"

extern "C" void app_main(void) {
  // Inicialización de datos globales
  dataInit();

  // Inicia la task de control y obtiene su handle
  TaskHandle_t control_task_handle = controlTaskStart();

  // Inicializa timer notify y apaga el timer al inicio
  timer_notify_init(control_task_handle);
  set_timer_state(false);

  // Inicializa task de comandos UART
  commandTaskInit();

  // Inicia task que lee la IMU
  sensorMeasureTaskStart();

  // Task RYUW
  uwbMeasureTaskStart();

  // Task EKF
  ekfTaskStart(50);
}
