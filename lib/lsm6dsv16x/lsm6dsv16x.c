#include "app/lib/lsm6dsv16x.h"
#include "lsm6dsv16x-pid/lsm6dsv16x_reg.h"
#include "platform_interface/platform_interface.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lsm6dsv16x, CONFIG_LSM6DSV16X_LOG_LEVEL);

static stmdev_ctx_t dev_ctx;
static uint8_t whoamI;

static lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
static int16_t data_raw_acceleration[3];
static double_t acceleration_mg[3];

static uint8_t drdy_event = 0;

void lsm6dsv16x_read_data_irq_handler(void)
{
	drdy_event = 1;
}

// Interrupt 1 init
static const struct gpio_dt_spec imu_int_1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(imuint1), gpios);

static struct gpio_callback imu_int_1_cb_data;

void imu_int_1_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	lsm6dsv16x_read_data_irq_handler();
    return;
}

void lsm6dsv16x_init()
{
	int res = attach_interrupt(imu_int_1, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE, &imu_int_1_cb_data, imu_int_1_cb);
	if (res != 0) {
		LOG_ERR("Error while attaching interrupt %i", res);
	}

	lsm6dsv16x_pin_int_route_t pin_int;
	lsm6dsv16x_reset_t rst;

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

	if (whoamI != LSM6DSV16X_ID) {
		LOG_ERR("Error while fetching WHO_AM_I register. Received %#02x, expected %#02x", whoamI, LSM6DSV16X_ID);
		return;
	}

	 /* Restore default configuration */
  	lsm6dsv16x_reset_set(&dev_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
	LOG_WRN("lsm6dsv16x_reset_set %i", res);
	do {
		lsm6dsv16x_reset_get(&dev_ctx, &rst);
	} while (rst != LSM6DSV16X_READY);

	/* Enable Block Data Update */
	lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	// dummy read
	lsm6dsv16x_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

	pin_int.drdy_xl = PROPERTY_ENABLE;
	pin_int.drdy_temp = PROPERTY_DISABLE;
	lsm6dsv16x_pin_int1_route_set(&dev_ctx, &pin_int);

	/* Set Output Data Rate.
	* Selected data rate have to be equal or greater with respect
	* with MLC data rate.
	*/
	lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
	/* Set full scale */
	lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_2g);

	/* Configure filtering chain */
	filt_settling_mask.drdy = PROPERTY_ENABLE;
	filt_settling_mask.irq_xl = PROPERTY_ENABLE;
	filt_settling_mask.irq_g = PROPERTY_ENABLE;
	lsm6dsv16x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
	lsm6dsv16x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dsv16x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV16X_XL_STRONG);
}

void lsm6dsv16x_irq() {
	int res;
	if (drdy_event) {
		lsm6dsv16x_data_ready_t drdy;

		drdy_event = 0;

		/* Read output only if new xl value is available */
		lsm6dsv16x_flag_data_ready_get(&dev_ctx, &drdy);

		if (drdy.drdy_xl) {
			/* Read acceleration field data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lsm6dsv16x_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
			acceleration_mg[0] =
			lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[0]);
			acceleration_mg[1] =
			lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[1]);
			acceleration_mg[2] =
			lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[2]);
			LOG_DBG("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f",
					acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		}
	}
}
