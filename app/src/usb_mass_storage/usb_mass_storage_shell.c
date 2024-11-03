#include <zephyr/kernel.h>
#include "usb_mass_storage.h"
#include <zephyr/shell/shell.h>

static int cmd_fit(const struct shell *sh, size_t argc, char **argv)
{
    int res;
    res = usb_mass_storage_create_fit_example_file();
    if (res) {
        shell_error(sh, "Failed to create FIT example file (%i)", res);
    } else {
        shell_print(sh, "FIT example file created");
    }
	return 0;
}

SHELL_CMD_REGISTER(fit, NULL, "Storage commands", cmd_fit);
