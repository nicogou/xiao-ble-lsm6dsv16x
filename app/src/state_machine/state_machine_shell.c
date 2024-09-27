#if CONFIG_XIAO_BLE_SHELL

#include <zephyr/kernel.h>
#include "state_machine.h"
#include <zephyr/shell/shell.h>

static int cmd_recording_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_START_RECORDING);
	return 0;
}

static int cmd_recording_start_data_forwarder(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_START_RECORDING_DATA_FORWARDER);
	return 0;
}

static int cmd_recording_start_impulse(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_START_RECORDING_IMPULSE);
	return 0;
}

static int cmd_recording_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_STOP_RECORDING);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_recording_start,
	SHELL_CMD(data_forwarder, NULL, "Start recording with data forwarder.", cmd_recording_start_data_forwarder),
	SHELL_CMD(impulse, NULL, "Start recording with impulse predictions.", cmd_recording_start_impulse),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_recording,
	SHELL_CMD(start, &sub_recording_start, "Start recording.", cmd_recording_start),
	SHELL_CMD(stop, NULL, "Stop recording.", cmd_recording_stop),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(recording, &sub_recording, "Recording commands", NULL);

static int cmd_calibrating_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_START_CALIBRATION);
	return 0;
}

static int cmd_calibrating_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_STOP_CALIBRATION);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_calibrating,
	SHELL_CMD(start, NULL, "Start recording.", cmd_calibrating_start),
	SHELL_CMD(stop, NULL, "Stop recording.", cmd_calibrating_stop),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(cal, &sub_calibrating, "Calibrating commands", NULL);

#endif
