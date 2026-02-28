#include "ControlTask/ControlTask.h"
#include "SensorMeasureTask/SensorMeasureTask.h"
#include "TimerNotifyTask/TimerNotifyTask.h"
#include "UARTCommandTask/UARTCommandTask.h"
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

  // Inicia task que lee los sensores (IMU por ahora)
  TaskHandle_t sensor_task_handle = sensorMeasureTaskStart();
}
