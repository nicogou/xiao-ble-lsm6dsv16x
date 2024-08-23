#if CONFIG_XIAO_BLE_SHELL

#include <zephyr/kernel.h>
#include "usb_mass_storage.h"
#include <zephyr/shell/shell.h>

static int cmd_storage_ls(const struct shell *sh, size_t argc, char **argv)
{
    usb_mass_storage_lsdir(argv[1]);

	return 0;
}

static int cmd_storage_create(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_storage,
	SHELL_CMD_ARG(ls, NULL, "List contents of dir (default is topdir)", cmd_storage_ls, 0, 1),
	SHELL_CMD_ARG(create, NULL, "Create an entry.", cmd_storage_create, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(storage, &sub_storage, "Storage commands", NULL);

#endif