#include <zephyr/storage/flash_map.h>

#define STORAGE_PARTITION		storage_partition
#define STORAGE_PARTITION_ID		FIXED_PARTITION_ID(STORAGE_PARTITION)

int usb_mass_storage_init();
int usb_mass_storage_lsdir(const char *path);
