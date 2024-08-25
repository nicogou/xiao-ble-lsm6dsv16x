#include "app/lib/lsm6dsv16x.h"
#include "lsm6dsv16x-pid/lsm6dsv16x_reg.h"
#include "platform_interface/platform_interface.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lsm6dsv16x, CONFIG_LSM6DSV16X_LOG_LEVEL);

static stmdev_ctx_t dev_ctx;
static uint8_t whoamI;

static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;

static struct k_work imu_work;

// Interrupt 1 init
static const struct gpio_dt_spec imu_int_1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(imuint1), gpios);

static struct gpio_callback imu_int_1_cb_data;

void imu_int_1_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&imu_work);
    return;
}

int lsm6dsv16x_start_acquisition()
{
	lsm6dsv16x_pin_int_route_t pin_int;

	/* Enable Block Data Update */
	lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set full scale */
	lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_2g);

	/*
	* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
	* stored in FIFO) to FIFO_WATERMARK samples
	*/
	lsm6dsv16x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
	/* Set FIFO batch XL/Gyro ODR to 12.5Hz */
	lsm6dsv16x_fifo_xl_batch_set(&dev_ctx, LSM6DSV16X_XL_BATCHED_AT_60Hz);
	/* Set FIFO mode to Stream mode (aka Continuous Mode) */
	lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);

	pin_int.fifo_th = PROPERTY_ENABLE;
	lsm6dsv16x_pin_int1_route_set(&dev_ctx, &pin_int);
	//lsm6dsv16x_pin_int2_route_set(&dev_ctx, &pin_int);

	/* Set Output Data Rate */
	lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_60Hz);
	lsm6dsv16x_fifo_timestamp_batch_set(&dev_ctx, LSM6DSV16X_TMSTMP_DEC_8);
	lsm6dsv16x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

	return 0;
}

int lsm6dsv16x_stop_acquisition()
{
	lsm6dsv16x_reset_t rst;
	/* Restore default configuration */
  	lsm6dsv16x_reset_set(&dev_ctx, LSM6DSV16X_GLOBAL_RST);
	do {
		lsm6dsv16x_reset_get(&dev_ctx, &rst);
	} while (rst != LSM6DSV16X_READY);

	return 0;
}

void lsm6dsv16x_init()
{
	int res = attach_interrupt(imu_int_1, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE, &imu_int_1_cb_data, imu_int_1_cb);
	if (res != 0) {
		LOG_ERR("Error while attaching interrupt %i", res);
	}

	k_work_init(&imu_work, lsm6dsv16x_irq);

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
	do {
		lsm6dsv16x_reset_get(&dev_ctx, &rst);
	} while (rst != LSM6DSV16X_READY);
}

void lsm6dsv16x_irq(struct k_work *item) {

	uint16_t num = 0;
    lsm6dsv16x_fifo_status_t fifo_status;

	/* Read watermark flag */
	lsm6dsv16x_fifo_status_get(&dev_ctx, &fifo_status);

	num = fifo_status.fifo_level;

	LOG_DBG("-- FIFO num %d", num);
	while (num--) {
        lsm6dsv16x_fifo_out_raw_t f_data;

        /* Read FIFO sensor value */
        lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &f_data);
        datax = (int16_t *)&f_data.data[0];
        datay = (int16_t *)&f_data.data[2];
        dataz = (int16_t *)&f_data.data[4];
        ts = (int32_t *)&f_data.data[0];

        switch (f_data.tag) {
			case LSM6DSV16X_XL_NC_TAG:
				LOG_DBG("ACC [mg]:\t%4.2f\t%4.2f\t%4.2f",
					(float) lsm6dsv16x_from_fs2_to_mg(*datax),
					(float) lsm6dsv16x_from_fs2_to_mg(*datay),
					(float) lsm6dsv16x_from_fs2_to_mg(*dataz));
				break;

			case LSM6DSV16X_TIMESTAMP_TAG:
				LOG_DBG("TIMESTAMP [ms] %d", *ts);
				break;

			default:
				break;
        }
    }
	LOG_DBG("------\n");
}
