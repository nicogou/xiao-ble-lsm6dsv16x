#include <zephyr/kernel.h>
#include <zephyr/bluetooth/uuid.h>

/**
 *  @brief GoPro Service UUID value
 */
#define BT_UUID_GOPRO_VAL 0xfea6
/**
 *  @brief GoPro Service
 */
#define BT_UUID_GOPRO \
	BT_UUID_DECLARE_16(BT_UUID_GOPRO_VAL)

#define BT_UUID_GOPRO_CMD_VAL 	BT_UUID_128_ENCODE(0xbc1c0001, 0xbffd, 0x4da5, 0xb41c, 0x2e3b89013d93)
#define BT_UUID_GOPRO_CMD 		BT_UUID_DECLARE_128(BT_UUID_GOPRO_CMD_VAL)

void gopro_bluetooth_init();
