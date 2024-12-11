#include "state_machine.h"
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>

#include <app/lib/lsm6dsv16bx.h>
#include <app/lib/xiao_smp_bluetooth.h>
#include <usb_mass_storage/usb_mass_storage.h>
#include <edge-impulse/impulse.h>
#include <emulator/emulator.h>
#include <ui/ui.h>

LOG_MODULE_REGISTER(state_machine, CONFIG_APP_LOG_LEVEL);

/* User defined object */
struct s_object {
    /* This must be first */
    struct smf_ctx ctx;

    /* Events */
    struct k_event smf_event;
    int32_t events;

    /* Other state specific data add here */
} s_obj;

/* Forward declaration of state table */
static const struct smf_state xiao_states[];
static xiao_state_t current_state;
static xiao_recording_state_t recording_state = {.sflp_enabled = false, .data_forwarder_enabled = false, .edge_impulse_enabled = false, .qvar_enabled = false, .emulation_enabled = false,};

void state_machine_timer_expired_work_handler(struct k_work *work)
{
	state_machine_post_event(XIAO_EVENT_SLEEP);
}
// Define a work for the system workqueue to process as battery can't be read from an interrupt.
K_WORK_DEFINE(timer_state_machine_expired_work, state_machine_timer_expired_work_handler);

void state_machine_timer_expired_cb()
{
    // Submit work to the system workqueue as battery can't be read from an interrupt (which a timer expiring is).
    k_work_submit(&timer_state_machine_expired_work);
}
// Defining a timer to make the battery measurement regularly.
K_TIMER_DEFINE(timer_state_machine, state_machine_timer_expired_cb, NULL);

/* State OFF */
static void off_entry(void *o)
{
    LOG_INF("Entering OFF state.");
	ui_set_rgb_off(); /* Turn off LED */
	smp_bluetooth_stop_advertising();

    current_state = OFF;
    /*  FSM configuration clears the EMB_FUNC_EN_A register (04h of the embedded function registers),
     *  so FSM needs to be configured before Significant Motion detection.
     */
    uint8_t fsm_algs_to_start[1] = {0};
	lsm6dsv16bx_start_fsm(fsm_algs_to_start, 1);

	lsm6dsv16bx_start_significant_motion_detection();
}

static void off_run(void *o)
{
    struct s_object *s = (struct s_object *)o;

    /* Change states on Button Press Event */
    if (s->events & XIAO_EVENT_WAKE_UP) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[IDLE]);
    } else {
        LOG_WRN("Unhandled event in OFF state.");
    }
}

static void off_exit(void *o)
{
	lsm6dsv16bx_reset();
}

/* State IDLE */
static void idle_entry(void *o)
{
    LOG_INF("Entering IDLE state.");
    current_state = IDLE;
    k_timer_start(&timer_state_machine, K_SECONDS(CONFIG_IDLE_STATE_TIMEOUT), K_NO_WAIT);
	ui_set_rgb_on(  /*Red*/0,
                    /*Green*/smp_bluetooth_connected() ? 0 : UI_COLOR_MAX,
                    /*Blue*/smp_bluetooth_connected() ? UI_COLOR_MAX : 0,
                    /*Blink (%)*/0,
                    /*Duration (s)*/1); /* Turn on LED */
	smp_bluetooth_start_advertising();
}

static void idle_run(void *o)
{
    struct s_object *s = (struct s_object *)o;

    /* Change states on Button Press Event */
    if (s->events & XIAO_EVENT_START_RECORDING) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[RECORDING]);
    } else if (s->events & XIAO_EVENT_SLEEP) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[OFF]);
    } else {
        LOG_WRN("Unhandled event in IDLE state.");
    }
}

static void idle_exit(void *o)
{
	k_timer_stop(&timer_state_machine);
}

/* State RECORDING */
static void recording_entry(void *o)
{
    LOG_INF("Entering RECORDING state.");
    current_state = RECORDING;
	int res;

	if (recording_state.emulation_enabled)
	{
		res = emulator_session_start();
		if (res < 0){
			LOG_ERR("Unable to start emulation (%i)", res);
			return;
		}
	} else {
		res = usb_mass_storage_create_session();
		if (res < 0) {
			LOG_ERR("Unable to create session (%i)", res);
		}

        res = usb_mass_storage_write_to_current_session(SESSION_FILE_HEADER_SIMPLE, strlen(SESSION_FILE_HEADER_SIMPLE));
        if (res != 0){
            LOG_ERR("Failed to write session header to session file (simple)");
        }
		if (recording_state.sflp_enabled || recording_state.data_forwarder_enabled)
		{
			res = usb_mass_storage_write_to_current_session(SESSION_FILE_HEADER_SFLP, strlen(SESSION_FILE_HEADER_SFLP));
			if (res != 0){
				LOG_ERR("Failed to write session header to session file (SFLP)");
			}
		}
        if (recording_state.qvar_enabled || recording_state.data_forwarder_enabled)
		{
			res = usb_mass_storage_write_to_current_session(SESSION_FILE_HEADER_QVAR, strlen(SESSION_FILE_HEADER_QVAR));
			if (res != 0){
				LOG_ERR("Failed to write session header to session file (QVar)");
			}
		}
        res = usb_mass_storage_write_to_current_session(SESSION_FILE_HEADER_NEWLINE, strlen(SESSION_FILE_HEADER_NEWLINE));
        if (res != 0){
            LOG_ERR("Failed to write session header to session file (Newline)");
        }

	    lsm6dsv16bx_start_acquisition(false, recording_state.sflp_enabled, recording_state.qvar_enabled);
	}

#ifdef CONFIG_EDGE_IMPULSE
	if (recording_state.edge_impulse_enabled)
	{
		impulse_start_predicting();
	}
#endif

	ui_rgb_t current_color = ui_get_rgb();
	ui_set_rgb_on(current_color.red, current_color.green, current_color.blue, 50, 1);
}

static void recording_run(void *o)
{
    struct s_object *s = (struct s_object *)o;

    /* Change states on Button Press Event */
    if (s->events & XIAO_EVENT_STOP_RECORDING) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[IDLE]);
    } else {
        LOG_WRN("Unhandled event in RECORDING state.");
    }
}

static void recording_exit(void *o)
{
#ifdef CONFIG_EDGE_IMPULSE
	if (recording_state.edge_impulse_enabled)
	{
		impulse_stop_predicting();
	}
#endif

	if (recording_state.emulation_enabled)
	{
		emulator_session_stop();
	} else {
		lsm6dsv16bx_reset();
		int res = usb_mass_storage_end_current_session();
		if (res) {
			LOG_ERR("Unable to end session (%i)", res);
		}
	}

	ui_rgb_t current_color = ui_get_rgb();
	ui_set_rgb_on(current_color.red, current_color.green, current_color.blue, 0, 1);
}

/* State CALIBRATING */
static void calibrating_entry(void *o)
{
    LOG_INF("Entering CALIBRATING state.");
    current_state = CALIBRATING;
	lsm6dsv16bx_start_calibration();
}

static void calibrating_run(void *o)
{
    struct s_object *s = (struct s_object *)o;

    /* Change states on Button Press Event */
    if (s->events & XIAO_EVENT_STOP_CALIBRATION) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[IDLE]);
    } else {
        LOG_WRN("Unhandled event in CALIBRATING state.");
    }
}

static void calibrating_exit(void *o)
{
	lsm6dsv16bx_reset();
}

xiao_state_t state_machine_current_state(void) {
    return current_state;
}

/* Populate state table */
static const struct smf_state xiao_states[] = {
    [OFF] = SMF_CREATE_STATE(off_entry, off_run, off_exit, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, idle_exit, NULL, NULL),
    [RECORDING] = SMF_CREATE_STATE(recording_entry, recording_run, recording_exit, NULL, NULL),
    [CALIBRATING] = SMF_CREATE_STATE(calibrating_entry, calibrating_run, calibrating_exit, NULL, NULL),
};

int state_machine_post_event(xiao_event_t event)
{
    /* Post the event */
    k_event_post(&s_obj.smf_event, event);
    return 0;
}

int state_machine_set_recording_state(xiao_recording_state_t state)
{
	recording_state = state;
	return 0;
}

xiao_recording_state_t state_machine_get_recording_state()
{
	return recording_state;
}

/* Initialize the state machine */
int state_machine_init(xiao_state_t starting_state)
{
    /* Initialize the event */
    k_event_init(&s_obj.smf_event);

    /* Set initial state */
    smf_set_initial(SMF_CTX(&s_obj), &xiao_states[starting_state]);

    return 0;
}

/* Run the state machine */
int state_machine_run(void)
{
    int ret;

    /* Run the state machine */
    while((s_obj.events = k_event_wait(&s_obj.smf_event, 0xFFFFFFFF, true, K_FOREVER))) {
        /* State machine terminates if a non-zero value is returned */
        ret = smf_run_state(SMF_CTX(&s_obj));
        if (ret) {
                /* handle return code and terminate state machine */
                break;
        }
    }
    return 0;
}
