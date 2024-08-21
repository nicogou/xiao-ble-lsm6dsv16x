#include "app/lib/lsm6dsv16x.h"
#include "lsm6dsv16x-pid/lsm6dsv16x_reg.h"
#include "platform_interface/platform_interface.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lsm6dsv16x, CONFIG_LSM6DSV16X_LOG_LEVEL);

static stmdev_ctx_t dev_ctx;
static uint8_t whoamI;

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static double_t acceleration_mg[3];
static double_t angular_rate_mdps[3];
static double_t temperature_degC;

static lsm6dsv16x_filt_settling_mask_t filt_settling_mask;

void lsm6dsv16x_init()
{
	//lsm6dsv16x_pin_int_route_t pin_int;
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

  /* Enable Block Data Update */
  lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_7Hz5);
  lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_15Hz);
  /* Set full scale */
  lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_2g);
  lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_2000dps);
  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv16x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv16x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv16x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV16X_GY_ULTRA_LIGHT);
  lsm6dsv16x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv16x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV16X_XL_STRONG);
}

void lsm6dsv16x_poll() {
	lsm6dsv16x_data_ready_t drdy;

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
		LOG_WRN("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f",
				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    }

    /* Read output only if new xl value is available */
    if (drdy.drdy_gy) {
		/* Read angular rate field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6dsv16x_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
		angular_rate_mdps[0] =
			lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] =
			lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] =
			lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[2]);
		LOG_WRN("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f",
				angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
    }

    if (drdy.drdy_temp) {
		/* Read temperature data */
		memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		lsm6dsv16x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
		temperature_degC = lsm6dsv16x_from_lsb_to_celsius(
							data_raw_temperature);
		LOG_WRN("Temperature [degC]:%6.2f", temperature_degC);
	}
}
