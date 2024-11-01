#if CONFIG_XIAO_BLE_SHELL

#include <zephyr/kernel.h>
#include "state_machine.h"
#include <zephyr/shell/shell.h>

static int cmd_recording_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	xiao_recording_state_t wanted_state = {.sflp_enabled = false, .data_forwarder_enabled = false, .edge_impulse_enabled = false, .qvar_enabled = false};
	for (int ii = 1; ii < argc; ii++)
	{
		if (strcmp(argv[ii], SFLP_STRING) == 0)
		{
			shell_print(sh, "SFLP enabled");
			wanted_state.sflp_enabled = true;
		}

		if (strcmp(argv[ii], DATA_FORWARDER_STRING) == 0)
		{
			shell_print(sh, "Data forwarder enabled");
			wanted_state.data_forwarder_enabled = true;
		}

		if (strcmp(argv[ii], EDGE_IMPULSE_STRING) == 0)
		{
			shell_print(sh, "Edge Impulse enabled");
			wanted_state.edge_impulse_enabled = true;
		}

		if (strcmp(argv[ii], QVAR_STRING) == 0)
		{
			shell_print(sh, "QVar enabled");
			wanted_state.qvar_enabled = true;
		}
	}
	state_machine_set_recording_state(wanted_state);

	state_machine_post_event(XIAO_EVENT_START_RECORDING);
	shell_print(sh, "%s", "Recording Start event posted");
	return 0;
}

static int cmd_recording_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_STOP_RECORDING);
	shell_print(sh, "%s", "Recording Stop event posted");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_recording,
	SHELL_CMD(start, NULL, "Start recording. Use sflp, data_forwarder, edge_impulse and/or gbias to enable corresponding options", cmd_recording_start),
	SHELL_CMD(stop, NULL, "Stop recording.", cmd_recording_stop),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(recording, &sub_recording, "Recording commands", NULL);

static int cmd_calibrating_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_START_CALIBRATION);
	shell_print(sh, "%s", "Calibration Start event posted");
	return 0;
}

static int cmd_calibrating_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	state_machine_post_event(XIAO_EVENT_STOP_CALIBRATION);
	shell_print(sh, "%s", "Calibration Stop event posted");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_calibrating,
	SHELL_CMD(start, NULL, "Start calibration.", cmd_calibrating_start),
	SHELL_CMD(stop, NULL, "Stop calibration.", cmd_calibrating_stop),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(cal, &sub_calibrating, "Calibrating commands", NULL);

#endif
