#pragma once

#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/fs.h>

#define STORAGE_PARTITION		storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION)
#define SESSION_DIR_NAME		"SESSION"
#define MAX_PATH				128

#define SESSION_FILE_NAME		"SESSION"
#define SESSION_FILE_EXTENSION	".CSV"
#define SESSION_FILE_HEADER		"ts,ax,ay,az,gx,gy,gz\n"

int usb_mass_storage_init();
int usb_mass_storage_lsdir(const char *path);
int usb_mass_storage_create_file(const char *path, const char *filename, struct fs_file_t *f, bool keep_open);
int usb_mass_storage_create_dir(const char *path);
int usb_mass_storage_write_to_file(char* data, size_t len, struct fs_file_t *f, bool erase_content);
int usb_mass_storage_close_file(struct fs_file_t *f);
int usb_mass_storage_create_session();
int usb_mass_storage_end_current_session();
int usb_mass_storage_write_to_current_session(char* data, size_t len);
