#include "state_machine.h"
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>

#include <app/lib/lsm6dsv16x.h>
#include <usb_mass_storage/usb_mass_storage.h>
#include <edge-impulse/impulse.h>

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

/* State IDLE */
static void idle_entry(void *o)
{
    LOG_INF("Entering IDLE state.");
    current_state = IDLE;
}

static void idle_run(void *o)
{
    struct s_object *s = (struct s_object *)o;

    /* Change states on Button Press Event */
    if (s->events & XIAO_EVENT_START_RECORDING) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[RECORDING]);
    } else if (s->events & XIAO_EVENT_START_RECORDING_DATA_FORWARDER) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[RECORDING_DATA_FORWARDER]);
    } else if (s->events & XIAO_EVENT_START_RECORDING_IMPULSE) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[RECORDING_IMPULSE]);
    } else if (s->events & XIAO_EVENT_START_CALIBRATION) {
        smf_set_state(SMF_CTX(&s_obj), &xiao_states[CALIBRATING]);
    } else {
        LOG_WRN("Unhandled event in IDLE state.");
    }
}

/* State RECORDING */
static void recording_entry(void *o)
{
    LOG_INF("Entering RECORDING state.");
    current_state = RECORDING;
	int res = usb_mass_storage_create_session();
	if (res < 0) {
		LOG_ERR("Unable to create session (%i)", res);
	}
    lsm6dsv16x_start_acquisition(false);
}

static void recording_simple_entry(void *o)
{
    LOG_INF("Entering RECORDING_SIMPLE state.");
    current_state = RECORDING_SIMPLE;
}

static void recording_sflp_entry(void *o)
{
    LOG_INF("Entering RECORDING_SFLP state.");
    current_state = RECORDING_SFLP;
}

static void data_forwarder_entry(void *o)
{
    LOG_INF("Entering RECORDING_DATA_FORWARDER state.");
    current_state = RECORDING_DATA_FORWARDER;
}

static void impulse_entry(void *o)
{
    LOG_INF("Entering RECORDING_IMPULSE state.");
    current_state = RECORDING_IMPULSE;
	impulse_start_predicting();
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

static void recording_simple_run(void *o)
{
	__unused struct s_object *s = (struct s_object *)o;

	/* Use smf_set_handled to handle recording_simple specific events.
	 * Events that are common to all recording states are handled in recording_run.
	 */
}

static void recording_sflp_run(void *o)
{
	__unused struct s_object *s = (struct s_object *)o;

	/* Use smf_set_handled to handle recording_simple specific events.
	 * Events that are common to all recording states are handled in recording_run.
	 */
}

static void recording_data_forwarder_run(void *o)
{
	__unused struct s_object *s = (struct s_object *)o;

	/* Use smf_set_handled to handle recording_data_forwarder specific events.
	 * Events that are common to all recording states are handled in recording_run.
	 */
}

static void recording_impulse_run(void *o)
{
	__unused struct s_object *s = (struct s_object *)o;

	/* Use smf_set_handled to handle recording_impulse specific events.
	 * Events that are common to all recording states are handled in recording_run.
	 */
}

static void recording_exit(void *o)
{
    lsm6dsv16x_stop_acquisition();
	int res = usb_mass_storage_end_current_session();
	if (res) {
		LOG_ERR("Unable to end session (%i)", res);
	}
}

static void recording_simple_exit(void *o)
{
}

static void recording_sflp_exit(void *o)
{
}

static void data_forwarder_exit(void *o)
{
}

static void impulse_exit(void *o)
{
	impulse_stop_predicting();
}

/* State CALIBRATING */
static void calibrating_entry(void *o)
{
    LOG_INF("Entering CALIBRATING state.");
    current_state = CALIBRATING;
	lsm6dsv16x_start_calibration();
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
	lsm6dsv16x_stop_calibration();
}

xiao_state_t state_machine_current_state(void) {
    return current_state;
}

/* Populate state table */
static const struct smf_state xiao_states[] = {
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL, NULL),
    [RECORDING] = SMF_CREATE_STATE(recording_entry, recording_run, recording_exit, NULL, &xiao_states[RECORDING_SIMPLE]),
    [RECORDING_SIMPLE] = SMF_CREATE_STATE(recording_simple_entry, recording_simple_run, recording_simple_exit, &xiao_states[RECORDING], NULL),
    [RECORDING_SFLP] = SMF_CREATE_STATE(recording_simple_entry, recording_simple_run, recording_simple_exit, &xiao_states[RECORDING], NULL),
    [RECORDING_DATA_FORWARDER] = SMF_CREATE_STATE(data_forwarder_entry, recording_data_forwarder_run, data_forwarder_exit, &xiao_states[RECORDING], NULL),
    [RECORDING_IMPULSE] = SMF_CREATE_STATE(impulse_entry, recording_impulse_run, impulse_exit, &xiao_states[RECORDING], NULL),
    [CALIBRATING] = SMF_CREATE_STATE(calibrating_entry, calibrating_run, calibrating_exit, NULL, NULL),
};

int state_machine_post_event(xiao_event_t event)
{
    /* Post the event */
    k_event_post(&s_obj.smf_event, event);
    return 0;
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
