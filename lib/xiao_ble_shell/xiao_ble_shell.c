#include <zephyr/shell/shell.h>

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

SHELL_CMD_REGISTER(uf2, NULL, "Switch Xiao BLE to UF2 bootloader mode.", cmd_uf2);

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
