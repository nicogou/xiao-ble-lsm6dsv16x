#if CONFIG_XIAO_BLE_SHELL

#include <zephyr/kernel.h>
#include "usb_mass_storage.h"
#include <zephyr/shell/shell.h>
#include <zephyr/fs/fs.h>

static struct fs_file_t file;

static int cmd_storage_ls(const struct shell *sh, size_t argc, char **argv)
{
    usb_mass_storage_lsdir(argv[1]);

	return 0;
}

static int cmd_storage_create(const struct shell *sh, size_t argc, char **argv)
{
    int res;
    res = usb_mass_storage_create_file(argv[2], argv[1], &file, false);
    if (res) {
        shell_error(sh, "Failed to create file %s at %s (%d)", argv[1], argv[2], res);
    } else {
        shell_print(sh, "File %s created at %s", argv[1], argv[2]);
    }
	return 0;
}

static int cmd_storage_mkdir(const struct shell *sh, size_t argc, char **argv)
{
    int res;
    res = usb_mass_storage_create_dir(argv[1]);
    if (res) {
        shell_error(sh, "Failed to create directory at %s (%d)", argv[1], res);
    } else {
        shell_print(sh, "Directory %s created", argv[1]);
    }
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_storage,
	SHELL_CMD_ARG(ls, NULL, "List contents of dir (default is topdir)\nstorage ls [path]", cmd_storage_ls, 0, 1),
	SHELL_CMD_ARG(create, NULL, "Create a file.\nstorage create <filename> <file_path>", cmd_storage_create, 2, 1),
	SHELL_CMD_ARG(mkdir, NULL, "Create a directory.\nstorage mkdir <dir_path>\nNote: the directory name can't have an underscore.", cmd_storage_mkdir, 2, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(storage, &sub_storage, "Storage commands", NULL);

#endif
