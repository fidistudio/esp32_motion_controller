#include "timer_task.h"
#include "timer_hw.h"

extern "C" void app_main(void)
{
    TaskHandle_t task = timer_task_start();
    timer_hw_init(task);
}
