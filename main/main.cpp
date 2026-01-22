#include "timer_task.h"
#include "timer_hw.h"
#include "uart_deco.h"
#include "data.h"

extern "C" void app_main(void)
{
    TaskHandle_t task = timer_task_start();
    timer_hw_init(task);
    set_timer_state(true);
    uart_deco_init();
}
