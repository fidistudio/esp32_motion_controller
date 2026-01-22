#include "data.h"

static bool time_enabled;

bool is_timer_enabled(void){
    return time_enabled;
}

void set_timer_state(bool state){
    time_enabled=state;
}