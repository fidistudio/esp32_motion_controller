#include "ControlTask/ControlTask.h"
#include "TimerNotifyTask/TimerNotifyTask.h"
#include "UARTCommandTask/UARTCommandTask.h"
#include "data.h"

extern "C" void app_main(void)
{
    dataInit();
    TaskHandle_t control_task_handle=controlTaskStart();
    timer_notify_init(control_task_handle);
    
    commandTaskInit();
}
