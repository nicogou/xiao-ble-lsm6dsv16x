#include <zephyr/shell/shell.h>
#include <date_time.h>
#include <stdlib.h>

static int cmd_demo_ping(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "pong");

	return 0;
}

static int cmd_demo_board(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, CONFIG_BOARD);

	return 0;
}

static int cmd_demo_params(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "argc = %zd", argc);
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(sh, "  argv[%zd] = %s", cnt, argv[cnt]);
	}

	return 0;
}

static int cmd_demo_hexdump(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "argc = %zd", argc);
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(sh, "argv[%zd]", cnt);
		shell_hexdump(sh, argv[cnt], strlen(argv[cnt]));
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_demo,
	SHELL_CMD(hexdump, NULL, "Hexdump params command.", cmd_demo_hexdump),
	SHELL_CMD(params, NULL, "Print params command.", cmd_demo_params),
	SHELL_CMD(ping, NULL, "Ping command.", cmd_demo_ping),
	SHELL_CMD(board, NULL, "Show board name command.", cmd_demo_board),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(demo, &sub_demo, "Demo commands", NULL);

static int cmd_uf2(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Switching Xiao BLE to UF2 bootloader mode in 3");
	k_msleep(500);
	shell_print(sh, "                                             2");
	k_msleep(500);
	shell_print(sh, "                                             1");
	NRF_POWER->GPREGRET = 0x57; // 0xA8 OTA, 0x4e Serial
	NVIC_SystemReset();

	return 0;
}

static int cmd_ota(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Switching Xiao BLE to OTA bootloader mode in 3");
	k_msleep(500);
	shell_print(sh, "                                             2");
	k_msleep(500);
	shell_print(sh, "                                             1");
	NRF_POWER->GPREGRET = 0xA8; // 0xA8 OTA, 0x4e Serial
	NVIC_SystemReset();

	return 0;
}

static int cmd_serial(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Switching Xiao BLE to Serial bootloader mode in 3");
	k_msleep(500);
	shell_print(sh, "                                             2");
	k_msleep(500);
	shell_print(sh, "                                             1");
	NRF_POWER->GPREGRET = 0x4E; // 0xA8 OTA, 0x4e Serial
	NVIC_SystemReset();

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uf2,
	SHELL_CMD(uf2, NULL, "Go into UF2 mode.", cmd_uf2),
	SHELL_CMD(ota, NULL, "Go into OTA mode.", cmd_ota),
	SHELL_CMD(serial, NULL, "Go into Serial mode.", cmd_serial),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(dfu, &sub_uf2, "Switch Xiao BLE to bootloader mode.", cmd_uf2);

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Resetting Xiao BLE...");
	k_msleep(10);
	NVIC_SystemReset();

	return 0;
}

SHELL_CMD_REGISTER(reset, NULL, "Reset Xiao BLE.", cmd_reset);

static int cmd_set_time(const struct shell *sh, size_t argc, char **argv)
{
	shell_warn(sh, "WARNING: this function sets the GMT time!");
	const struct tm now = {.tm_hour = strtol(argv[1], NULL, 10), .tm_min = strtol(argv[2], NULL, 10), .tm_sec = strtol(argv[3], NULL, 10), .tm_mday = strtol(argv[4], NULL, 10), .tm_mon = strtol(argv[5], NULL, 10) - 1, .tm_year = strtol(argv[6], NULL, 10) - 1900};
	int res = date_time_set(&now);
	if (res)
	{
		shell_error(sh, "Formatting error while setting time!");
		return res;
	}

	int64_t t;
	res = date_time_now(&t);
	if (res)
	{
		shell_error(sh, "Cannot get current time (%i)", res);
		return res;
	}
	shell_print(sh, "Current time %llu", t);

	return 0;
}

static int cmd_get_time(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int64_t t;
	int res = date_time_now(&t);
	if (res)
	{
		shell_error(sh, "Error while retrieving time (%i)", res);
		return res;
	}

	shell_print(sh, "Current time %llu", t);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_date,
	SHELL_CMD_ARG(set, NULL, "Set GMT time. Format: time set hour(0-23) minutes(0-59) seconds(0-59) day(1-31) month(1-12) year(>2015)", cmd_set_time, 7, 0),
	SHELL_CMD(get, NULL, "Get GMT time.", cmd_get_time),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(time, &sub_date, "Get/Set Xiao BLE time.", NULL);
