#pragma once

#include <zephyr/sys/util_macro.h>

/* List of events */
typedef enum {
    XIAO_EVENT_START_RECORDING = BIT(0),
    XIAO_EVENT_STOP_RECORDING = BIT(1),
} xiao_event_t;

/* List of states */
typedef enum xiao_state { IDLE, RECORDING } xiao_state_t;

int state_machine_init(void);
int state_machine_run(void);
int state_machine_post_event(xiao_event_t event);
xiao_state_t state_machine_current_state(void);