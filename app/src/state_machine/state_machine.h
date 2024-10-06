#pragma once

#include <zephyr/sys/util_macro.h>

/* List of events */
typedef enum {
    XIAO_EVENT_START_RECORDING = BIT(0),
    XIAO_EVENT_START_RECORDING_SFLP = BIT(1),
    XIAO_EVENT_START_RECORDING_DATA_FORWARDER = BIT(2),
    XIAO_EVENT_START_RECORDING_IMPULSE = BIT(3),
    XIAO_EVENT_STOP_RECORDING = BIT(4),
    XIAO_EVENT_START_CALIBRATION = BIT(5),
    XIAO_EVENT_STOP_CALIBRATION = BIT(6),
	XIAO_EVENT_START_EMULATION = BIT(7),
	XIAO_EVENT_STOP_EMULATION = BIT(8),
} xiao_event_t;

/* List of states */
typedef enum xiao_state {
	IDLE,
	RECORDING,
		RECORDING_SIMPLE,
		RECORDING_SFLP,
		RECORDING_DATA_FORWARDER,
		RECORDING_IMPULSE,
	CALIBRATING,
	EMULATING,
} xiao_state_t;

int state_machine_init(xiao_state_t starting_state);
int state_machine_run(void);
int state_machine_post_event(xiao_event_t event);
xiao_state_t state_machine_current_state(void);
