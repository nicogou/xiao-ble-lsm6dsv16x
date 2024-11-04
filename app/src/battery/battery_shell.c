#include <zephyr/kernel.h>
#include "battery.h"
#include <zephyr/shell/shell.h>

static int cmd_battery_read(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Battery level: %u%%", battery_read()/100);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_battery,
	SHELL_CMD(read, NULL, "Read battery level.", cmd_battery_read),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(battery, &sub_battery, "Battery commands", NULL);
