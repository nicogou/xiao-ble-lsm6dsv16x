#include "usb_mass_storage.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <stdio.h>
#include <ff.h>
#include <errno.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(mass_storage, CONFIG_APP_LOG_LEVEL);

static struct fs_mount_t fs_mnt;
struct fs_file_t current_session_file;
static struct fs_file_t calibration_file;
static int current_session_nb = 0;

struct fs_file_t* usb_mass_storage_get_session_file_p()
{
	return &current_session_file;
}

struct fs_file_t* usb_mass_storage_get_calibration_file_p()
{
	return &calibration_file;
}

static int setup_flash(struct fs_mount_t *mnt)
{
	int rc = 0;
	unsigned int id;
	const struct flash_area *pfa;

	mnt->storage_dev = (void *)STORAGE_PARTITION_ID;
	id = STORAGE_PARTITION_ID;

	rc = flash_area_open(id, &pfa);
	LOG_DBG("Area %u at 0x%x on %s for %u bytes",
	       id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
	       (unsigned int)pfa->fa_size);

	if (rc < 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		LOG_INF("Erasing flash area ... ");
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		if (rc != 0) {
			LOG_ERR("Failed to erase falsh area (%i)", rc);
		}
	}

	if (rc < 0) {
		flash_area_close(pfa);
	}
	return rc;
}

static int mount_app_fs(struct fs_mount_t *mnt)
{
	int rc;

	static FATFS fat_fs;

	mnt->type = FS_FATFS;
	mnt->fs_data = &fat_fs;
	mnt->mnt_point = MOUNT_POINT;

	rc = fs_mount(mnt);

	return rc;
}

static void setup_disk(void)
{
	struct fs_mount_t *mp = &fs_mnt;
	struct fs_statvfs sbuf;
	int rc;

	rc = setup_flash(mp);
	if (rc < 0) {
		LOG_ERR("Failed to setup flash area");
		return;
	}

	if (!IS_ENABLED(CONFIG_FAT_FILESYSTEM_ELM)) {
		LOG_INF("No compatible file system selected");
		return;
	}

	rc = mount_app_fs(mp);
	if (rc < 0) {
		LOG_ERR("Failed to mount filesystem");
		return;
	}

	/* Allow log messages to flush to avoid interleaved output */
	k_sleep(K_MSEC(50));

	LOG_DBG("Mount %s: %d", fs_mnt.mnt_point, rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		LOG_ERR("FAIL: statvfs: %d", rc);
		return;
	}

	LOG_DBG("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu",
	       mp->mnt_point,
	       sbuf.f_bsize, sbuf.f_frsize,
	       sbuf.f_blocks, sbuf.f_bfree);

	return;
}

/* List dir entry by path
 *
 * @param path Absolute path to list
 *
 * @return Negative errno code on error, number of listed entries on
 *         success.
 */
int usb_mass_storage_lsdir(const char *path)
{
	struct fs_mount_t *mp = &fs_mnt;
	int rc;
	struct fs_dir_t dir;
	int count = 0;

	const char** tmp_path;
	if (path == NULL) {
		tmp_path = &mp->mnt_point;
	} else {
		if (memcmp(path, mp->mnt_point, strlen(mp->mnt_point))) {
			LOG_ERR("Path %s is not in mount point %s", path, mp->mnt_point);
			return -EINVAL;
		}
		tmp_path = &path;
	}

	fs_dir_t_init(&dir);

	/* Verify fs_opendir() */
	rc = fs_opendir(&dir, *tmp_path);
	if (rc) {
		LOG_ERR("Error opening dir %s [%d]", *tmp_path, rc);
		return rc;
	}

	LOG_DBG("Listing dir %s ...", *tmp_path);
	while (rc >= 0) {
		struct fs_dirent ent = { 0 };

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			LOG_ERR("Failed to read directory entries");
			break;
		}
		if (ent.name[0] == 0) {
			LOG_DBG("End of files");
			break;
		}
		LOG_DBG("  %c %u %s",
		       (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
		       ent.size,
		       ent.name);
	}

	/* Verify fs_closedir() */
	fs_closedir(&dir);
	if (rc == 0) {
		rc = count;
	}

	return rc;
}

int usb_mass_storage_create_file(const char *path, const char *filename, struct fs_file_t *f, bool keep_open){
	char file_path[128];
	uint8_t base = 0;
	struct fs_mount_t *mp = &fs_mnt;

	if (path == NULL) {
		memcpy(file_path, mp->mnt_point, strlen(mp->mnt_point));
		base = strlen(mp->mnt_point);
	} else {
		base = strlen(path);
		if (path[strlen(path) - 1] == '/') {
			base = strlen(path) - 1;
		}
		memcpy(file_path, path, base);
	}

	file_path[base++] = '/';
	file_path[base] = 0;

	strcat(file_path, filename);
	fs_file_t_init(f);

	int rc = fs_open(f, file_path, FS_O_CREATE | FS_O_RDWR);
	if (rc != 0) {
		LOG_ERR("Failed to create file %s (%i)", file_path, rc);
		return rc;
	}

	if (!keep_open) {
		rc = fs_close(f);
		if (rc != 0) {
			LOG_ERR("Failed to close file %s (%i)", file_path, rc);
			return rc;
		}
	}

	return 0;
}

int usb_mass_storage_write_to_file(char* data, size_t len, struct fs_file_t *f, bool erase_content)
{
	int res;
	if (erase_content)
	{
		res = fs_truncate(f, 0);
		if (res != 0)
		{
			LOG_ERR("Error truncating calibration file (%i)", res);
		}

	}

	res = fs_write(f, data, strlen(data)); // Write data to corresponding SD file.
	if (res < 0) {
		LOG_ERR("Failed to write data to file (%i)", res);
		return res;
	}
	if (res < len)
	{
		LOG_WRN("The data has not been properly written to the file (written data length in bytes: %i vs expected %u - errno %i)", res, len, errno);
		return -ENOMEM;
	}

	return 0;
}

int usb_mass_storage_close_file(struct fs_file_t *f)
{
	int res = fs_close(f);
	if (res != 0) {
		LOG_WRN("Unable to close file (%i)", res);
		return res;
	}
	return 0;
}

int usb_mass_storage_create_dir(const char *path){
	char file_path[128];
	uint8_t base = 0;

	if (path == NULL) {
		LOG_ERR("No path specified to create directory");
		return -EINVAL;
	}

	memcpy(file_path, path, strlen(path));
	if (path[strlen(path) - 1] == '/') {
		file_path[strlen(path) - 1] = 0;
	}
	base = strlen(file_path);

	int res = fs_mkdir(path);
	if (res != 0) {
		LOG_ERR("Failed to create dir %s", path);
		return res;
	}

	return 0;
}

static int get_session_nb(const char *path){
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_ERR("Error opening dir %s [%d]", path, res);
		return res;
	}

	int session_nb = 0;
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		/* In that case, create session_0 dir. */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			if (strncmp(entry.name, SESSION_DIR_NAME, strlen(SESSION_DIR_NAME)) == 0) {
				errno = 0;
				uintmax_t tmp = strtoumax(&entry.name[strlen(SESSION_DIR_NAME)], NULL, 10); // Get directory number as uint
				if (tmp == UINTMAX_MAX && errno == ERANGE){
					LOG_ERR("Could not find directory number.");
				} else {
					if (tmp > session_nb) {
						session_nb = tmp;
					}
				}
			}
		} else {
		}
		count++;
	}

	/* Verify fs_closedir() */
	res = fs_closedir(&dirp);
	if (res != 0) {
		LOG_ERR("Error while closing dir");
		return res;
	}

	// At this point, session_nb is equal to the highest dir number.
	// Increment it to create new folder.
	session_nb++;
	return session_nb;
}

int usb_mass_storage_create_session()
{
	struct fs_mount_t *mp = &fs_mnt;

	char path[MAX_PATH];
	int base = 0;

	int nb = get_session_nb(mp->mnt_point);
	if (nb < 0){
		LOG_ERR("Unable to get session number.");
		return nb;
	}
	int length = snprintf( NULL, 0, "%d", nb);
	char* nb_str = malloc(length + 1 + strlen(SESSION_DIR_NAME));
	snprintf(nb_str, length + 1 + strlen(SESSION_DIR_NAME), "%s%d", SESSION_DIR_NAME, nb);

	strncpy(path, mp->mnt_point, sizeof(path));
	base += strlen(mp->mnt_point);

	path[base++] = '/';
	path[base] = 0;

	strcat(path, nb_str);
	base += strlen(nb_str);
	free(nb_str);

	int res = usb_mass_storage_create_dir(path);
	if (res != 0){
		LOG_ERR("Failed to create dir %s", path);
		return res;
	}

	res = usb_mass_storage_create_file(path, SESSION_FILE_NAME SESSION_FILE_EXTENSION, &current_session_file, true);
	if (res != 0) {
		LOG_ERR("Failed to create data_file %s (%i)", path, res);
		return res;
	}

	res = usb_mass_storage_write_to_current_session(SESSION_FILE_HEADER, strlen(SESSION_FILE_HEADER));
	if (res != 0){
		LOG_ERR("Failed to write session header to session file");
		return res;
	}

	current_session_nb = nb;

	return nb;
}

int usb_mass_storage_end_current_session(){
	int res = fs_close(&current_session_file);
	if (res != 0) {
		LOG_WRN("Unable to close acc file (%i)", res);
		return res;
	}
	return 0;
}

int usb_mass_storage_write_to_current_session(char* data, size_t len){
	int res = fs_write(&current_session_file, data, strlen(data)); // Write data to corresponding SD file.
	if (res < 0) {
		LOG_ERR("Failed to write data to current session file (%i)", res);
		return res;
	}
	if (res < len)
	{
		LOG_WRN("The data has not been properly written to the session (written data length in bytes: %i vs expected %u - errno %i)", res, len, errno);
		return -ENOMEM;
	}

	return 0;
}

int usb_mass_storage_check_calibration_file_contents(float *x, float *y, float *z)
{
	struct fs_dirent file_info;
	int ret = fs_stat(MOUNT_POINT "/" CALIBRATION_FILE_NAME, &file_info);
	if (ret) {
		LOG_ERR("Failed to get calibration file info (%i)", ret);
		return ret;
	}

	if (file_info.size != CALIBRATION_FILE_SIZE)
	{
		LOG_ERR("Calibration file is not the right size. Expected %u, got %u", CALIBRATION_FILE_SIZE, file_info.size);
		return -ENOENT;
	}

	struct fs_file_t f;
	char f_path[] = MOUNT_POINT "/" CALIBRATION_FILE_NAME;
	fs_file_t_init(&f);
	ret = fs_open(&f, f_path, FS_O_READ);
	if (ret != 0) {
		LOG_ERR("Failed to open file %s (%i)", f_path, ret);
		return ret;
	}

	char file_content[CALIBRATION_FILE_SIZE];
	int size_read = fs_read(&f, file_content, CALIBRATION_FILE_SIZE);
	if (size_read != CALIBRATION_FILE_SIZE && size_read >= 0)
	{
		LOG_ERR("Calibration file contents read improperly. Expected %u bytes, got %i", CALIBRATION_FILE_SIZE, size_read);
		return -ENOENT;
	}
	else if (size_read < 0)
	{
		LOG_ERR("Failed to read calibration file %i", size_read);
		return size_read;
	}

	char xf[CALIBRATION_DATA_SIZE + 1], yf[CALIBRATION_DATA_SIZE + 1], zf[CALIBRATION_DATA_SIZE + 1];
	memcpy(xf, &file_content[CALIBRATION_DATA_X_POSITION], CALIBRATION_DATA_SIZE);
	xf[CALIBRATION_DATA_SIZE] = 0;
	*x = strtof(xf, NULL);
	memcpy(yf, &file_content[CALIBRATION_DATA_Y_POSITION], CALIBRATION_DATA_SIZE);
	yf[CALIBRATION_DATA_SIZE] = 0;
	*y = strtof(yf, NULL);
	memcpy(zf, &file_content[CALIBRATION_DATA_Z_POSITION], CALIBRATION_DATA_SIZE);
	z[CALIBRATION_DATA_SIZE] = 0;
	*z = strtof(zf, NULL);

	return 0;
}

int usb_mass_storage_init() {

	setup_disk();

#if CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT == 0
	int ret = usb_enable(NULL);

	if (ret != 0) {
		LOG_ERR("Failed to enable USB (%i)", ret);
		return -EIO;
	}
#endif

	return 0;
}
