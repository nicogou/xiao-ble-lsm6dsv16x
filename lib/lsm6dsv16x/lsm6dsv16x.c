#include "app/lib/lsm6dsv16x.h"
#include "lsm6dsv16x_reg.h"
#include "platform_interface/platform_interface.h"
#include "lsm6dsv16x_sflp_utils.h"

#include <zephyr/logging/log.h>

#if (!defined(CONFIG_LSM6DSV16X_SPI) && !defined(CONFIG_LSM6DSV16X_I2C))
#error "No communication interface defined. Enable either CONFIG_LSM6DSV16X_SPI or CONFIG_LSM6DSV16X_I2C."
#elif (defined(CONFIG_LSM6DSV16X_SPI) && defined(CONFIG_LSM6DSV16X_I2C))
#error "Too many communication interfaces defined. Enable either CONFIG_LSM6DSV16X_SPI or CONFIG_LSM6DSV16X_I2C, but not both."
#endif

LOG_MODULE_REGISTER(lsm6dsv16x, CONFIG_LSM6DSV16X_LOG_LEVEL);

static lsm6dsv16x_sensor_t sensor;
static lsm6dsv16x_sflp_gbias_t gbias = {.gbias_x = 0, .gbias_y = 0, .gbias_z = 0};
static lsm6dsv16x_ah_qvar_mode_t qvar_mode;

static struct k_work imu_int1_work;

// Interrupt 1 init
static const struct gpio_dt_spec imu_int_1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(imuint1), gpios);

static struct gpio_callback imu_int_1_cb_data;

void imu_int_1_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&imu_int1_work);
    return;
}

static struct k_work imu_int2_work;

// Interrupt 2 init
static const struct gpio_dt_spec imu_int_2 =
    GPIO_DT_SPEC_GET(DT_ALIAS(imuint2), gpios);

static struct gpio_callback imu_int_2_cb_data;

void imu_int_2_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&imu_int2_work);
    return;
}

static void _calibration_timer_cb(struct k_timer *dummy)
{
	sensor.state = LSM6DSV16X_CALIBRATION_RECORDING;
}

K_TIMER_DEFINE(calibration_timer, _calibration_timer_cb, NULL);

int lsm6dsv16x_start_acquisition(bool enable_gbias, bool enable_sflp, bool enable_qvar)
{
	lsm6dsv16x_pin_int_route_t pin1_int = {0};
	lsm6dsv16x_pin_int_route_t pin2_int = {0};
	lsm6dsv16x_fifo_sflp_raw_t fifo_sflp = {0};
	lsm6dsv16x_filt_settling_mask_t filt_settling_mask = {0};
	int ret;

	/* Enable Block Data Update */
	ret = lsm6dsv16x_block_data_update_set(&sensor.dev_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("lsm6dsv16x_block_data_update_set (%i)", ret);
	}
	/* Set full scale */
	ret = lsm6dsv16x_xl_full_scale_set(&sensor.dev_ctx, LSM6DSV16X_2g);
	if (ret) {
		LOG_ERR("lsm6dsv16x_xl_full_scale_set (%i)", ret);
	}
	ret = lsm6dsv16x_gy_full_scale_set(&sensor.dev_ctx, LSM6DSV16X_2000dps);
	if (ret) {
		LOG_ERR("lsm6dsv16x_gy_full_scale_set (%i)", ret);
	}

	/*
	* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
	* stored in FIFO) to FIFO_WATERMARK samples
	*/
	ret = lsm6dsv16x_fifo_watermark_set(&sensor.dev_ctx, FIFO_WATERMARK);
	if (ret) {
		LOG_ERR("lsm6dsv16x_fifo_watermark_set (%i)", ret);
	}

	/* Set FIFO batch of sflp data */
	fifo_sflp.game_rotation = enable_sflp;
	fifo_sflp.gravity = enable_sflp;
	fifo_sflp.gbias = enable_gbias;
	ret = lsm6dsv16x_fifo_sflp_batch_set(&sensor.dev_ctx, fifo_sflp);
	if (ret) {
		LOG_ERR("lsm6dsv16x_fifo_sflp_batch_set (%i)", ret);
	}

	/* Set FIFO batch XL/Gyro ODR to 60Hz */
	ret = lsm6dsv16x_fifo_xl_batch_set(&sensor.dev_ctx, LSM6DSV16X_XL_BATCHED_AT_60Hz);
	if (ret) {
		LOG_ERR("lsm6dsv16x_fifo_xl_batch_set (%i)", ret);
	}
	ret = lsm6dsv16x_fifo_gy_batch_set(&sensor.dev_ctx, LSM6DSV16X_GY_BATCHED_AT_60Hz);
	if (ret) {
		LOG_ERR("lsm6dsv16x_fifo_gy_batch_set (%i)", ret);
	}
	/* Set FIFO mode to Stream mode (aka Continuous Mode) */
	ret = lsm6dsv16x_fifo_mode_set(&sensor.dev_ctx, LSM6DSV16X_STREAM_MODE);
	if (ret) {
		LOG_ERR("lsm6dsv16x_fifo_mode_set (%i)", ret);
	}

	pin1_int.fifo_th = PROPERTY_ENABLE;
	ret = lsm6dsv16x_pin_int1_route_set(&sensor.dev_ctx, &pin1_int);
	if (ret) {
		LOG_ERR("lsm6dsv16x_pin_int1_route_set (%i)", ret);
	}

	/* Set Output Data Rate */
	ret = lsm6dsv16x_xl_data_rate_set(&sensor.dev_ctx, LSM6DSV16X_ODR_AT_960Hz);
	if (ret) {
		LOG_ERR("lsm6dsv16x_xl_data_rate_set (%i)", ret);
	}
	ret = lsm6dsv16x_gy_data_rate_set(&sensor.dev_ctx, LSM6DSV16X_ODR_AT_960Hz);
	if (ret) {
		LOG_ERR("lsm6dsv16x_gy_data_rate_set (%i)", ret);
	}
	ret = lsm6dsv16x_sflp_data_rate_set(&sensor.dev_ctx, LSM6DSV16X_SFLP_60Hz);
	if (ret) {
		LOG_ERR("lsm6dsv16x_sflp_data_rate_set (%i)", ret);
	}
	ret = lsm6dsv16x_fifo_timestamp_batch_set(&sensor.dev_ctx, LSM6DSV16X_TMSTMP_DEC_1);
	if (ret) {
		LOG_ERR("lsm6dsv16x_fifo_timestamp_batch_set (%i)", ret);
	}
	ret = lsm6dsv16x_timestamp_set(&sensor.dev_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("lsm6dsv16x_timestamp_set (%i)", ret);
	}
	ret = lsm6dsv16x_sflp_game_rotation_set(&sensor.dev_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("lsm6dsv16x_sflp_game_rotation_set (%i)", ret);
	}

	ret = lsm6dsv16x_sflp_game_gbias_set(&sensor.dev_ctx, &gbias);
	if (ret) {
		LOG_ERR("lsm6dsv16x_sflp_game_gbias_set (%i)", ret);
	}

	sensor.state = LSM6DSV16X_RECORDING;
	/* Mask accelerometer and gyroscope data until the settling of the sensors filter is completed */
  	filt_settling_mask.drdy = PROPERTY_ENABLE;
  	filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  	filt_settling_mask.irq_g = PROPERTY_ENABLE;
  	ret = lsm6dsv16x_filt_settling_mask_set(&sensor.dev_ctx, filt_settling_mask);
	if (ret) {
		LOG_ERR("lsm6dsv16x_filt_settling_mask_set (%i)", ret);
	}
//	lsm6dsv16x_filt_xl_lp2_set(&sensor.dev_ctx, PROPERTY_ENABLE);
//	lsm6dsv16x_filt_xl_lp2_bandwidth_set(&sensor.dev_ctx, LSM6DSV16X_XL_STRONG);

	if (enable_qvar)
	{
		qvar_mode.ah_qvar_en = 1;
		ret = lsm6dsv16x_ah_qvar_mode_set(&sensor.dev_ctx, qvar_mode);
		if (ret) {
			LOG_ERR("lsm6dsv16x_ah_qvar_mode_set (%i)", ret);
		}

		pin2_int.drdy_ah_qvar = PROPERTY_ENABLE;
		ret = lsm6dsv16x_pin_int2_route_set(&sensor.dev_ctx, &pin2_int);
		if (ret) {
			LOG_ERR("lsm6dsv16x_pin_int2_route_set (%i)", ret);
		}
	}

	sensor.nb_samples_to_discard = CONFIG_LSM6DSV16X_SAMPLES_TO_DISCARD;

	return 0;
}

int lsm6dsv16x_stop_acquisition()
{
	lsm6dsv16x_reset_t rst;
	/* Restore default configuration */
  	lsm6dsv16x_reset_set(&sensor.dev_ctx, LSM6DSV16X_GLOBAL_RST);
	do {
		lsm6dsv16x_reset_get(&sensor.dev_ctx, &rst);
	} while (rst != LSM6DSV16X_READY);

	sensor.state = LSM6DSV16X_IDLE;
	return 0;
}

int lsm6dsv16x_start_calibration()
{
	int res = lsm6dsv16x_start_acquisition(true, false, false);
	if (res != 0)
	{
		LOG_ERR("Error while starting the sensor");
		return res;
	}

	sensor.state = LSM6DSV16X_CALIBRATION_SETTLING;
	k_timer_start(&calibration_timer, K_SECONDS(CONFIG_LSM6DSV16X_CALIBRATION_SETTLING_TIME), K_NO_WAIT);
	return 0;
}

int lsm6dsv16x_stop_calibration() {
	// lsm6dsv16x_stop_acquisition switches the sensor state back to LSM6DSV16X_IDLE, no need to do it here.
	return lsm6dsv16x_stop_acquisition();
}

void lsm6dsv16x_set_gbias(float x, float y, float z)
{
	gbias.gbias_x = x * 1000.0f;
	gbias.gbias_y = y * 1000.0f;
	gbias.gbias_z = z * 1000.0f;
}

void lsm6dsv16x_int2_irq(struct k_work *item) {
	lsm6dsv16x_all_sources_t all_sources;
	int16_t data;

	/* Read output only if new values are available */
    lsm6dsv16x_all_sources_get(&sensor.dev_ctx, &all_sources);
    if (all_sources.drdy_ah_qvar) {
      lsm6dsv16x_ah_qvar_raw_get(&sensor.dev_ctx, &data);

	  LOG_DBG("QVAR [mV]:%6.2f", (double)lsm6dsv16x_from_lsb_to_mv(data));
    }
}

void lsm6dsv16x_init(lsm6dsv16x_cb_t cb)
{
	sensor.callbacks = cb;

	int res = attach_interrupt(imu_int_1, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE, &imu_int_1_cb_data, imu_int_1_cb);
	if (res != 0) {
		LOG_ERR("Error while attaching interrupt 1 %i", res);
	}

	res = attach_interrupt(imu_int_2, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE, &imu_int_2_cb_data, imu_int_2_cb);
	if (res != 0) {
		LOG_ERR("Error while attaching interrupt 2 %i", res);
	}

	k_work_init(&imu_int1_work, lsm6dsv16x_int1_irq);
	k_work_init(&imu_int2_work, lsm6dsv16x_int2_irq);

	lsm6dsv16x_reset_t rst;

	/* Initialize mems driver interface */
	sensor.dev_ctx.write_reg = platform_write;
	sensor.dev_ctx.read_reg = platform_read;
	sensor.dev_ctx.mdelay = platform_delay;
	sensor.dev_ctx.handle = NULL;

	/* Init test platform */
	platform_init(sensor.dev_ctx.handle);
	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);
	/* Check device ID */
	lsm6dsv16x_device_id_get(&sensor.dev_ctx, &sensor.whoamI);

	if (sensor.whoamI != LSM6DSV16X_ID) {
		LOG_ERR("Error while fetching WHO_AM_I register. Received %#02x, expected %#02x", sensor.whoamI, LSM6DSV16X_ID);
		return;
	}

	 /* Restore default configuration */
  	lsm6dsv16x_reset_set(&sensor.dev_ctx, LSM6DSV16X_GLOBAL_RST);
	do {
		lsm6dsv16x_reset_get(&sensor.dev_ctx, &rst);
	} while (rst != LSM6DSV16X_READY);

	sensor.state = LSM6DSV16X_IDLE;
}

static void _data_handler_recording(lsm6dsv16x_fifo_out_raw_t* f_data)
{
	float quat[4];
	int16_t *datax = (int16_t *)&f_data->data[0];
	int16_t *datay = (int16_t *)&f_data->data[2];
	int16_t *dataz = (int16_t *)&f_data->data[4];
	int32_t *ts = (int32_t *)&f_data->data[0];

	switch (f_data->tag) {
		case LSM6DSV16X_XL_NC_TAG:
			if (sensor.callbacks.lsm6dsv16x_acc_sample_cb) {
				(*sensor.callbacks.lsm6dsv16x_acc_sample_cb)(lsm6dsv16x_from_fs2_to_mg(*datax), lsm6dsv16x_from_fs2_to_mg(*datay), lsm6dsv16x_from_fs2_to_mg(*dataz));
			}
			break;

		case LSM6DSV16X_GY_NC_TAG:
			if (sensor.callbacks.lsm6dsv16x_gyro_sample_cb) {
				float_t argx = lsm6dsv16x_from_fs2000_to_mdps(*datax) - (gbias.gbias_x) / 1000.0f;
				float_t argy = lsm6dsv16x_from_fs2000_to_mdps(*datay) - (gbias.gbias_y) / 1000.0f;
				float_t argz = lsm6dsv16x_from_fs2000_to_mdps(*dataz) - (gbias.gbias_z) / 1000.0f;
				(*sensor.callbacks.lsm6dsv16x_gyro_sample_cb)(argx, argy, argz);
			}
			break;

		case LSM6DSV16X_TIMESTAMP_TAG:
			if (sensor.callbacks.lsm6dsv16x_ts_sample_cb) {
				(*sensor.callbacks.lsm6dsv16x_ts_sample_cb)(lsm6dsv16x_from_lsb_to_nsec(*ts));
			}
			break;

		case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
			if (sensor.callbacks.lsm6dsv16x_gbias_sample_cb) {
				(*sensor.callbacks.lsm6dsv16x_gbias_sample_cb)(lsm6dsv16x_from_fs125_to_mdps(*datax), lsm6dsv16x_from_fs125_to_mdps(*datay), lsm6dsv16x_from_fs125_to_mdps(*dataz));
			}
			break;

		case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
			if (sensor.callbacks.lsm6dsv16x_gravity_sample_cb) {
				(*sensor.callbacks.lsm6dsv16x_gravity_sample_cb)(lsm6dsv16x_from_sflp_to_mg(*datax), lsm6dsv16x_from_sflp_to_mg(*datay), lsm6dsv16x_from_sflp_to_mg(*dataz));
			}
			break;

		case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
			sflp2q(quat, (uint16_t *)&f_data->data[0]);
			if (sensor.callbacks.lsm6dsv16x_game_rot_sample_cb) {
				(*sensor.callbacks.lsm6dsv16x_game_rot_sample_cb)(quat[0], quat[1], quat[2], quat[3]);
			}
			break;

		default:
			LOG_WRN("Unhandled data (tag %u) received in FIFO", f_data->tag);
			break;
	}
}

static bool _data_handler_calibrating(lsm6dsv16x_fifo_out_raw_t* f_data, float_t* res)
{
	int16_t *datax = (int16_t *)&f_data->data[0];
	int16_t *datay = (int16_t *)&f_data->data[2];
	int16_t *dataz = (int16_t *)&f_data->data[4];
	bool handled = false;

	switch (f_data->tag) {
		case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
			res[0] = lsm6dsv16x_from_fs125_to_mdps(*datax);
			res[1] = lsm6dsv16x_from_fs125_to_mdps(*datay);
			res[2] = lsm6dsv16x_from_fs125_to_mdps(*dataz);
			handled = true;
			break;

		default:
			//LOG_WRN("Unhandled data (tag %u) received while calibrating IMU", f_data->tag);
			break;
	}

	return handled;
}

void lsm6dsv16x_int1_irq(struct k_work *item) {

	uint16_t num = 0;
    lsm6dsv16x_fifo_status_t fifo_status;
	float_t gbias_tmp[3];
	bool res = false;

	/* Read watermark flag */
	lsm6dsv16x_fifo_status_get(&sensor.dev_ctx, &fifo_status);
	num = fifo_status.fifo_level;

	if (sensor.state != LSM6DSV16X_CALIBRATION_SETTLING) {
		LOG_DBG("Received %d samples from FIFO.", num);
	}

	while (num--) {
        lsm6dsv16x_fifo_out_raw_t f_data;

        /* Read FIFO sensor value */
        lsm6dsv16x_fifo_out_raw_get(&sensor.dev_ctx, &f_data);

		if (sensor.nb_samples_to_discard) {
			sensor.nb_samples_to_discard--;
			continue;
		}

		if (sensor.state == LSM6DSV16X_RECORDING)
		{
			_data_handler_recording(&f_data);
		} else if (sensor.state == LSM6DSV16X_CALIBRATION_RECORDING)
		{
			res = _data_handler_calibrating(&f_data, gbias_tmp);
			if (res)
			{
				break;
			}
		}
	}

	if (res && sensor.state == LSM6DSV16X_CALIBRATION_RECORDING && sensor.callbacks.lsm6dsv16x_calibration_result_cb)
	{
		lsm6dsv16x_stop_calibration();
		(*sensor.callbacks.lsm6dsv16x_calibration_result_cb)(gbias_tmp[0], gbias_tmp[1], gbias_tmp[2]);
	}
}
