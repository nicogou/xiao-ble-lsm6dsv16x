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

static int cmd_recording_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_STOP_RECORDING);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_recording,
	SHELL_CMD(start, NULL, "Start recording.", cmd_recording_start),
	SHELL_CMD(stop, NULL, "Stop recording.", cmd_recording_stop),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(recording, &sub_recording, "Recording commands", NULL);

#endif