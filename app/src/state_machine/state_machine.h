#pragma once
#include <zephyr/kernel.h>
#include <zephyr/sys/util_macro.h>

/* List of events */
typedef enum {
    XIAO_EVENT_START_RECORDING = BIT(0),
    XIAO_EVENT_STOP_RECORDING = BIT(1),
    XIAO_EVENT_START_CALIBRATION = BIT(2),
    XIAO_EVENT_STOP_CALIBRATION = BIT(3),
    XIAO_EVENT_WAKE_UP = BIT(4),
    XIAO_EVENT_SLEEP = BIT(5),
} xiao_event_t;

#define SFLP_STRING "sflp"
#define DATA_FORWARDER_STRING "data_forwarder"
#define EDGE_IMPULSE_STRING "edge_impulse"
#define QVAR_STRING "qvar"
#define EMULATION_STRING "emulation"

typedef struct
{
	bool sflp_enabled;
	bool data_forwarder_enabled;
	bool edge_impulse_enabled;
	bool qvar_enabled;
	bool emulation_enabled;
} xiao_recording_state_t;

/* List of states */
typedef enum xiao_state {
	OFF,
	IDLE,
	RECORDING,
	CALIBRATING,
	EMULATING,
} xiao_state_t;

int state_machine_init(xiao_state_t starting_state);
int state_machine_run(void);
int state_machine_post_event(xiao_event_t event);
xiao_state_t state_machine_current_state(void);
int state_machine_set_recording_state(xiao_recording_state_t state);
xiao_recording_state_t state_machine_get_recording_state();
