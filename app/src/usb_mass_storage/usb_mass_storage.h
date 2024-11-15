#pragma once

#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/fs.h>

#define MOUNT_POINT "/NAND:"

#define STORAGE_PARTITION		storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION)
#define SESSION_DIR_NAME		"SESSION"
#define MAX_PATH				128

#define SESSION_FILE_NAME		"SESSION"
#define SESSION_FILE_EXTENSION	".CSV"
#define SESSION_FILE_HEADER_SIMPLE		"ts,ax,ay,az,gx,gy,gz\n"
#define SESSION_FILE_HEADER_SFLP		"ts,ax,ay,az,gx,gy,gz,grotx,groty,grotz,grotw,gravx,gravy,gravz\n"
#define SESSION_FILE_NB_COLUMN_SIMPLE	7
#define SESSION_FILE_NB_COLUMN_SFLP		14

#define CALIBRATION_FILE_NAME	"CAL.TXT"
#define CALIBRATION_FILE_SIZE	29
#define CALIBRATION_DATA_SIZE	7
#define CALIBRATION_DATA_X_POSITION	2
#define CALIBRATION_DATA_Y_POSITION	12
#define CALIBRATION_DATA_Z_POSITION	22

#define SESSION_WR_BUFFER_SIZE 2048
#define SESSION_WR_BUFFER_THRESHOLD 1024

int usb_mass_storage_init();
int usb_mass_storage_lsdir(const char *path);
int usb_mass_storage_create_file(const char *path, const char *filename, struct fs_file_t *f, bool keep_open);
int usb_mass_storage_create_dir(const char *path);
int usb_mass_storage_write_to_file(char* data, size_t len, struct fs_file_t *f, bool erase_content);
int usb_mass_storage_get_session_header(const char* data, struct fs_file_t *f);
int usb_mass_storage_read_line(char* data, size_t len, size_t offset, struct fs_file_t *f);
int usb_mass_storage_close_file(struct fs_file_t *f);
int usb_mass_storage_create_session();
int usb_mass_storage_end_current_session();
int usb_mass_storage_write_to_current_session(char* data, size_t len);
int usb_mass_storage_check_calibration_file_contents(float *x, float *y, float *z);
struct fs_file_t* usb_mass_storage_get_session_file_p();
struct fs_file_t* usb_mass_storage_get_calibration_file_p();
int usb_mass_storage_create_fit_example_file();
