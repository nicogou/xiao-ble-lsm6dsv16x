#include "app/lib/lsm6dsv16x.h"
#include "lsm6dsv16x-pid/lsm6dsv16x_reg.h"
#include "platform_interface/platform_interface.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lsm6dsv16x, CONFIG_LSM6DSV16X_LOG_LEVEL);

static stmdev_ctx_t dev_ctx;
static uint8_t whoamI;

void lsm6dsv16x_init()
{
	//lsm6dsv16x_pin_int_route_t pin_int;
	//lsm6dsv16x_reset_t rst;

	/* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.mdelay = platform_delay;
	dev_ctx.handle = NULL;

	/* Init test platform */
	platform_init(dev_ctx.handle);
	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);
	/* Check device ID */
	lsm6dsv16x_device_id_get(&dev_ctx, &whoamI);

	LOG_INF("Who Am I : %.x - %.x", whoamI, LSM6DSV16X_ID);
}
