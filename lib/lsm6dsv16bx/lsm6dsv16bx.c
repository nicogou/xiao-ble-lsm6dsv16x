#include "app/lib/lsm6dsv16bx.h"
#include "lsm6dsv16bx_reg.h"
#include "platform_interface/platform_interface.h"
#include "lsm6dsv16bx_sflp_utils.h"
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#if (!defined(CONFIG_LSM6DSV16BX_SPI) && !defined(CONFIG_LSM6DSV16BX_I2C))
#error "No communication interface defined. Enable either CONFIG_LSM6DSV16BX_SPI or CONFIG_LSM6DSV16BX_I2C."
#elif (defined(CONFIG_LSM6DSV16BX_SPI) && defined(CONFIG_LSM6DSV16BX_I2C))
#error "Too many communication interfaces defined. Enable either CONFIG_LSM6DSV16BX_SPI or CONFIG_LSM6DSV16BX_I2C, but not both."
#endif

LOG_MODULE_REGISTER(lsm6dsv16bx, CONFIG_LSM6DSV16BX_LOG_LEVEL);

static lsm6dsv16bx_sensor_t sensor;
static lsm6dsv16bx_sflp_gbias_t gbias = {.gbias_x = 0, .gbias_y = 0, .gbias_z = 0};
static lsm6dsv16bx_ah_qvar_mode_t qvar_mode;

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

#if DT_NODE_EXISTS(imu_int2)
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
#endif

static void _calibration_timer_cb(struct k_timer *dummy);
K_TIMER_DEFINE(calibration_timer, _calibration_timer_cb, NULL);

void calibration_timer_work_cb(struct k_work *work)
{
	switch (sensor.state.calib)
	{
	case LSM6DSV16BX_CALIBRATION_SETTLING:
		// Start recording. Launch timer to see if calibration times out.
		sensor.state.calib = LSM6DSV16BX_CALIBRATION_RECORDING;
		k_timer_start(&calibration_timer, K_SECONDS(CONFIG_LSM6DSV16BX_CALIBRATION_TIMEOUT), K_NO_WAIT);
		break;

	case LSM6DSV16BX_CALIBRATION_RECORDING:
		// If timer times out while calibration recording, we stop the calibration.
		LOG_WRN("Calibration timed out, exiting calibration.");
		if (sensor.callbacks.lsm6dsv16bx_calibration_result_cb)
		{
			(*sensor.callbacks.lsm6dsv16bx_calibration_result_cb)(false, 0.0f, 0.0f, 0.0f);
		} else {
			LOG_ERR("No Calibration callback defined!");
		}
		break;

	default:
		LOG_WRN("Unhandled calibration timer callback...");
		break;
	}
}

K_WORK_DEFINE(calibration_timer_work, calibration_timer_work_cb);

void _calibration_timer_cb(struct k_timer *dummy)
{
	k_work_submit(&calibration_timer_work);
}

int lsm6dsv16bx_start_acquisition(bool enable_gbias, bool enable_sflp, bool enable_qvar)
{
	lsm6dsv16bx_pin_int_route_t pin1_int = {0};
	lsm6dsv16bx_fifo_sflp_raw_t fifo_sflp = {0};
	lsm6dsv16bx_filt_settling_mask_t filt_settling_mask = {0};
	int ret;

	/* Enable Block Data Update */
	ret = lsm6dsv16bx_block_data_update_set(&sensor.dev_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_block_data_update_set (%i)", ret);
	}
	/* Set full scale */
	ret = lsm6dsv16bx_xl_full_scale_set(&sensor.dev_ctx, sensor.scale.xl_scale);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_xl_full_scale_set (%i)", ret);
	}
	ret = lsm6dsv16bx_gy_full_scale_set(&sensor.dev_ctx, LSM6DSV16BX_2000dps);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_gy_full_scale_set (%i)", ret);
	}

	/*
	* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
	* stored in FIFO) to FIFO_WATERMARK samples
	*/
	ret = lsm6dsv16bx_fifo_watermark_set(&sensor.dev_ctx, FIFO_WATERMARK);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_fifo_watermark_set (%i)", ret);
	}

	/* Set FIFO batch of sflp data */
	fifo_sflp.game_rotation = enable_sflp;
	fifo_sflp.gravity = enable_sflp;
	fifo_sflp.gbias = enable_gbias;
	ret = lsm6dsv16bx_fifo_sflp_batch_set(&sensor.dev_ctx, fifo_sflp);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_fifo_sflp_batch_set (%i)", ret);
	}

	uint8_t xl_sampling_frequency = LSM6DSV16BX_XL_BATCHED_AT_120Hz;
	uint8_t g_sampling_frequency = LSM6DSV16BX_XL_BATCHED_AT_120Hz;
	uint8_t sflp_sampling_frequency = LSM6DSV16BX_SFLP_120Hz;
	if (!enable_sflp && !enable_gbias)
	{
		xl_sampling_frequency = LSM6DSV16BX_XL_BATCHED_AT_240Hz;
		g_sampling_frequency = LSM6DSV16BX_XL_BATCHED_AT_240Hz;
	}
	/* Set FIFO batch XL/Gyro ODR to specified frequency */
	ret = lsm6dsv16bx_fifo_xl_batch_set(&sensor.dev_ctx, xl_sampling_frequency);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_fifo_xl_batch_set (%i)", ret);
	}
	ret = lsm6dsv16bx_fifo_gy_batch_set(&sensor.dev_ctx, g_sampling_frequency);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_fifo_gy_batch_set (%i)", ret);
	}
	/* Set FIFO mode to Stream mode (aka Continuous Mode) */
	ret = lsm6dsv16bx_fifo_mode_set(&sensor.dev_ctx, LSM6DSV16BX_STREAM_MODE);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_fifo_mode_set (%i)", ret);
	}

	pin1_int.fifo_th = PROPERTY_ENABLE;
	ret = lsm6dsv16bx_pin_int1_route_set(&sensor.dev_ctx, pin1_int);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_pin_int1_route_set (%i)", ret);
	}

	/* Set Output Data Rate */
	ret = lsm6dsv16bx_xl_data_rate_set(&sensor.dev_ctx, LSM6DSV16BX_XL_ODR_AT_960Hz);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_xl_data_rate_set (%i)", ret);
	}
	ret = lsm6dsv16bx_gy_data_rate_set(&sensor.dev_ctx, LSM6DSV16BX_GY_ODR_AT_960Hz);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_gy_data_rate_set (%i)", ret);
	}
	if (enable_sflp || enable_gbias)
	{
		ret = lsm6dsv16bx_sflp_data_rate_set(&sensor.dev_ctx, sflp_sampling_frequency);
		if (ret) {
			LOG_ERR("lsm6dsv16bx_sflp_data_rate_set (%i)", ret);
		}
	}
	ret = lsm6dsv16bx_fifo_timestamp_batch_set(&sensor.dev_ctx, LSM6DSV16BX_TMSTMP_DEC_1);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_fifo_timestamp_batch_set (%i)", ret);
	}
	ret = lsm6dsv16bx_timestamp_set(&sensor.dev_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_timestamp_set (%i)", ret);
	}
	ret = lsm6dsv16bx_sflp_game_rotation_set(&sensor.dev_ctx, PROPERTY_ENABLE);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_sflp_game_rotation_set (%i)", ret);
	}

	ret = lsm6dsv16bx_sflp_game_gbias_set(&sensor.dev_ctx, &gbias);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_sflp_game_gbias_set (%i)", ret);
	}

	sensor.state.xl_enabled = true;
	sensor.state.gy_enabled = true;
	/* Mask accelerometer and gyroscope data until the settling of the sensors filter is completed */
  	filt_settling_mask.drdy = PROPERTY_ENABLE;
  	filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  	filt_settling_mask.irq_g = PROPERTY_ENABLE;
  	ret = lsm6dsv16bx_filt_settling_mask_set(&sensor.dev_ctx, filt_settling_mask);
	if (ret) {
		LOG_ERR("lsm6dsv16bx_filt_settling_mask_set (%i)", ret);
	}
//	lsm6dsv16bx_filt_xl_lp2_set(&sensor.dev_ctx, PROPERTY_ENABLE);
//	lsm6dsv16bx_filt_xl_lp2_bandwidth_set(&sensor.dev_ctx, LSM6DSV16BX_XL_STRONG);

	if (enable_qvar)
	{
		// Enable QVar data.
		qvar_mode.ah_qvar1_en = 1;
		ret = lsm6dsv16bx_ah_qvar_mode_set(&sensor.dev_ctx, qvar_mode);
		if (ret) {
			LOG_ERR("lsm6dsv16bx_ah_qvar_mode_set (%i)", ret);
		}

		// Enable QVar data in FIFO.
		lsm6dsv16bx_counter_bdr_reg1_t qvar_bdr;
		ret = lsm6dsv16bx_read_reg(&sensor.dev_ctx, LSM6DSV16BX_COUNTER_BDR_REG1, (uint8_t *)&qvar_bdr, 1);
		qvar_bdr.ah_qvar_batch_en = PROPERTY_ENABLE;
		if (ret) {
			LOG_ERR("lsm6dsv16bx_read_reg LSM6DSV16BX_COUNTER_BDR_REG1 (%i)", ret);
		}

		ret = lsm6dsv16bx_write_reg(&sensor.dev_ctx, LSM6DSV16BX_COUNTER_BDR_REG1, (uint8_t *)&qvar_bdr, 1);
		if (ret) {
			LOG_ERR("lsm6dsv16bx_write_reg LSM6DSV16BX_COUNTER_BDR_REG1 (%i)", ret);
		}

		sensor.state.qvar_enabled = true;
	}

	sensor.nb_samples_to_discard = CONFIG_LSM6DSV16BX_SAMPLES_TO_DISCARD;

	return 0;
}

int lsm6dsv16bx_reset()
{
	lsm6dsv16bx_reset_t rst;
	/* Restore default configuration */
  	lsm6dsv16bx_reset_set(&sensor.dev_ctx, LSM6DSV16BX_GLOBAL_RST);
	do {
		lsm6dsv16bx_reset_get(&sensor.dev_ctx, &rst);
	} while (rst != LSM6DSV16BX_READY);

	lsm6dsv16bx_state_t tmp_state = {
		.xl_enabled = false,
		.gy_enabled = false,
		.sflp_state = {
			.gbias_enabled = false,
			.game_rot_enabled = false,
			.gravity_enabled = false,
		},
		.sigmot_enabled = false,
		.fsm_enabled = false,
		.calib = LSM6DSV16BX_CALIBRATION_NOT_CALIBRATING,
		.int2_on_int1 = false,
	};
	memcpy(&sensor.state, &tmp_state, sizeof(tmp_state));

	return 0;
}

int lsm6dsv16bx_start_calibration()
{
	int res = lsm6dsv16bx_start_acquisition(true, false, false);
	if (res != 0)
	{
		LOG_ERR("Error while starting the sensor");
		return res;
	}

	sensor.state.calib = LSM6DSV16BX_CALIBRATION_SETTLING;
	k_timer_start(&calibration_timer, K_SECONDS(CONFIG_LSM6DSV16BX_CALIBRATION_SETTLING_TIME), K_NO_WAIT);
	return 0;
}

int lsm6dsv16bx_start_significant_motion_detection()
{
	lsm6dsv16bx_pin_int_route_t pin_int = { 0 };

	/* Enable Block Data Update */
	lsm6dsv16bx_block_data_update_set(&sensor.dev_ctx, PROPERTY_ENABLE);

	lsm6dsv16bx_sigmot_mode_set(&sensor.dev_ctx, 1);

	pin_int.sig_mot = PROPERTY_ENABLE;
	lsm6dsv16bx_pin_int1_route_set(&sensor.dev_ctx, pin_int);

	//lsm6dsv16bx_embedded_int_cfg_set(&sensor.dev_ctx, LSM6DSV16BX_INT_LATCH_ENABLE);

	/* Set Output Data Rate.*/
	lsm6dsv16bx_xl_data_rate_set(&sensor.dev_ctx, LSM6DSV16BX_XL_ODR_AT_120Hz);
	/* Set full scale */
	lsm6dsv16bx_xl_full_scale_set(&sensor.dev_ctx, LSM6DSV16BX_2g/*sensor.scale.xl_scale*/);

	sensor.state.sigmot_enabled = true;
	return 0;
}

/* fsm_alg_nb is an array containing the index of the algorithm to enable, n is the number of algorithm enabled.
 * Nothing is done to ensure compatibility between FSM algorithms. It is the responsibility of the application
 * to ensure that algorithms are compatible between them.
 */
int lsm6dsv16bx_start_fsm(uint8_t* fsm_alg_nb, uint8_t n)
{
	for (int ii = 0; ii < n; ii++)
	{
		if (fsm_alg_nb[ii] > LSM6DSV16BX_FSM_ALG_MAX_NB)
		{
			LOG_ERR("FSM Algorithm number must be between 0 and 7! %u", fsm_alg_nb[ii]);
			return -EINVAL;
		}
	}

	for (int ii = 0; ii < n; ii++)
	{
		if (sensor.fsm_configs.fsm_pre_cfg_cbs[fsm_alg_nb[ii]])
		{
			int res = (*sensor.fsm_configs.fsm_pre_cfg_cbs[fsm_alg_nb[ii]])(sensor.dev_ctx);
			if (res)
			{
				LOG_ERR("Pre config function for algorithm n°%u returned an error! %i", fsm_alg_nb[ii], res);
				return res;
			}
		}
	}

	/* Start Finite State Machine configuration */
	for (int ii = 0; ii < n; ii++)
	{
		for (int jj = 0; jj < (sensor.fsm_configs.fsm_ucf_cfg_size[fsm_alg_nb[ii]] / sizeof(ucf_line_t)); jj++ ) {
			lsm6dsv16bx_write_reg(&sensor.dev_ctx, sensor.fsm_configs.fsm_ucf_cfg[fsm_alg_nb[ii]][jj].address,
							(uint8_t *)&sensor.fsm_configs.fsm_ucf_cfg[fsm_alg_nb[ii]][jj].data, 1);
		}
	}

	sensor.state.fsm_enabled = true;
	return 0;
}

void lsm6dsv16bx_set_gbias(float x, float y, float z)
{
	// gbias needs to be in dps for lsm6dsv16bx_sflp_game_gbias_set
	gbias.gbias_x = x / 1000.0f;
	gbias.gbias_y = y / 1000.0f;
	gbias.gbias_z = z / 1000.0f;
}

void lsm6dsv16bx_int2_irq(struct k_work *item)
{
	bool handled = false;

	if (sensor.state.sigmot_enabled)
	{
		handled = true;

		lsm6dsv16bx_all_sources_t status;

		/* Read output only if new xl value is available */
		lsm6dsv16bx_all_sources_get(&sensor.dev_ctx, &status);

		if (status.sig_mot)
		{
			if (sensor.callbacks.lsm6dsv16bx_sigmot_cb) {
				(*sensor.callbacks.lsm6dsv16bx_sigmot_cb)();
			}
		}
	}

	if (sensor.state.fsm_enabled)
	{
		handled = true;

		lsm6dsv16bx_all_sources_t status;
		lsm6dsv16bx_fsm_out_t fsm_out;

		/* Read output only if new xl value is available */
		lsm6dsv16bx_all_sources_get(&sensor.dev_ctx, &status);

		if (status.fsm1 || status.fsm2 || status.fsm3 || status.fsm4 || status.fsm5 || status.fsm6 || status.fsm7 || status.fsm8) {
			lsm6dsv16bx_fsm_out_get(&sensor.dev_ctx, &fsm_out);
		}

		if (status.fsm1) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[0]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[0])(fsm_out.fsm_outs1);
			}
		}

		if (status.fsm2) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[1]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[1])(fsm_out.fsm_outs2);
			}
		}

		if (status.fsm3) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[2]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[2])(fsm_out.fsm_outs3);
			}
		}

		if (status.fsm4) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[3]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[3])(fsm_out.fsm_outs4);
			}
		}

		if (status.fsm5) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[4]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[4])(fsm_out.fsm_outs5);
			}
		}

		if (status.fsm6) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[5]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[5])(fsm_out.fsm_outs6);
			}
		}

		if (status.fsm7) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[6]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[6])(fsm_out.fsm_outs7);
			}
		}

		if (status.fsm8) {
			if (sensor.callbacks.lsm6dsv16bx_fsm_cbs[7]) {
				(*sensor.callbacks.lsm6dsv16bx_fsm_cbs[7])(fsm_out.fsm_outs8);
			}
		}
	}

	if (!handled)
	{
		LOG_WRN("IMU interrupt 2 fired, but not handled!");
	}
}

static void _data_handler_recording(lsm6dsv16bx_fifo_out_raw_t* f_data)
{
	float quat[4];
	int16_t *datax = (int16_t *)&f_data->data[0];
	int16_t *datay = (int16_t *)&f_data->data[2];
	int16_t *dataz = (int16_t *)&f_data->data[4];
	int32_t *ts = (int32_t *)&f_data->data[0];

	switch (f_data->tag) {
		case LSM6DSV16BX_XL_NC_TAG:
			if (sensor.callbacks.lsm6dsv16bx_acc_sample_cb) {
				(*sensor.callbacks.lsm6dsv16bx_acc_sample_cb)(	(*sensor.scale.xl_conversion_function)(*datax),
																(*sensor.scale.xl_conversion_function)(*datay),
																(*sensor.scale.xl_conversion_function)(*dataz));
			} else {
				LOG_ERR("No Accelerometer callback defined!");
			}
			break;

		case LSM6DSV16BX_GY_NC_TAG:
			if (sensor.callbacks.lsm6dsv16bx_gyro_sample_cb) {
				float_t argx = (*sensor.scale.gy_conversion_function)(*datax) - (gbias.gbias_x) * 1000.0f;
				float_t argy = (*sensor.scale.gy_conversion_function)(*datay) - (gbias.gbias_y) * 1000.0f;
				float_t argz = (*sensor.scale.gy_conversion_function)(*dataz) - (gbias.gbias_z) * 1000.0f;
				(*sensor.callbacks.lsm6dsv16bx_gyro_sample_cb)(argx, argy, argz);
			} else {
				LOG_ERR("No Gyroscope callback defined!");
			}
			break;

		case LSM6DSV16BX_TIMESTAMP_TAG:
			if (sensor.callbacks.lsm6dsv16bx_ts_sample_cb) {
				(*sensor.callbacks.lsm6dsv16bx_ts_sample_cb)(lsm6dsv16bx_from_lsb_to_nsec(*ts));
			} else {
				LOG_ERR("No Timestamp Bias callback defined!");
			}
			break;

		case LSM6DSV16BX_SFLP_GYROSCOPE_BIAS_TAG:
			if (sensor.callbacks.lsm6dsv16bx_gbias_sample_cb) {
				// Gyroscope bias is always at +/-125dps sensitivity (see AN5763 bottom of page 104 §9.6.3)
				(*sensor.callbacks.lsm6dsv16bx_gbias_sample_cb)(lsm6dsv16bx_from_fs125_to_mdps(*datax), lsm6dsv16bx_from_fs125_to_mdps(*datay), lsm6dsv16bx_from_fs125_to_mdps(*dataz));
			} else {
				LOG_ERR("No Gyroscope Bias callback defined!");
			}
			break;

		case LSM6DSV16BX_SFLP_GRAVITY_VECTOR_TAG:
			// Gravity vector is always at +/-2g sensitivity (see AN5763 bottom of page 104 §9.6.3)
			if (sensor.callbacks.lsm6dsv16bx_gravity_sample_cb) {
				(*sensor.callbacks.lsm6dsv16bx_gravity_sample_cb)(lsm6dsv16bx_from_sflp_to_mg(*datax), lsm6dsv16bx_from_sflp_to_mg(*datay), lsm6dsv16bx_from_sflp_to_mg(*dataz));
			} else {
				LOG_ERR("No Gravity callback defined!");
			}
			break;

		case LSM6DSV16BX_SFLP_GAME_ROTATION_VECTOR_TAG:
			sflp2q(quat, (uint16_t *)&f_data->data[0]);
			if (sensor.callbacks.lsm6dsv16bx_game_rot_sample_cb) {
				(*sensor.callbacks.lsm6dsv16bx_game_rot_sample_cb)(quat[0], quat[1], quat[2], quat[3]);
			} else {
				LOG_ERR("No Game Rotation callback defined!");
			}
			break;

		case LSM6DSV16BX_AH_QVAR:
			if (sensor.callbacks.lsm6dsv16bx_qvar_sample_cb) {
				(*sensor.callbacks.lsm6dsv16bx_qvar_sample_cb)(lsm6dsv16bx_from_lsb_to_mv(*datax));
			} else {
				LOG_ERR("No QVar callback defined!");
			}
			break;

		default:
			LOG_WRN("Unhandled data (tag %u) received in FIFO", f_data->tag);
			break;
	}
}

static bool _data_handler_calibrating(lsm6dsv16bx_fifo_out_raw_t* f_data, float_t* res)
{
	int16_t *datax = (int16_t *)&f_data->data[0];
	int16_t *datay = (int16_t *)&f_data->data[2];
	int16_t *dataz = (int16_t *)&f_data->data[4];
	bool handled = false;

	switch (f_data->tag) {
		case LSM6DSV16BX_SFLP_GYROSCOPE_BIAS_TAG:
			res[0] = lsm6dsv16bx_from_fs125_to_mdps(*datax);
			res[1] = lsm6dsv16bx_from_fs125_to_mdps(*datay);
			res[2] = lsm6dsv16bx_from_fs125_to_mdps(*dataz);
			handled = true;
			break;

		default:
			//LOG_WRN("Unhandled data (tag %u) received while calibrating IMU", f_data->tag);
			break;
	}

	return handled;
}

void lsm6dsv16bx_int1_irq(struct k_work *item)
{
	bool handled = false;
	uint16_t num = 0;
    lsm6dsv16bx_fifo_status_t fifo_status;
	float_t gbias_tmp[3];
	bool calibration_result = false;

	if (sensor.state.xl_enabled || sensor.state.gy_enabled || sensor.state.qvar_enabled)
	{
		handled = true;
		/* Read watermark flag */
		lsm6dsv16bx_fifo_status_get(&sensor.dev_ctx, &fifo_status);
		num = fifo_status.fifo_level;

		if (sensor.state.calib != LSM6DSV16BX_CALIBRATION_SETTLING) {
			LOG_DBG("Received %d samples from FIFO.", num);
		}

		while (num--) {
			lsm6dsv16bx_fifo_out_raw_t f_data;

			/* Read FIFO sensor value */
			lsm6dsv16bx_fifo_out_raw_get(&sensor.dev_ctx, &f_data);

			if (sensor.nb_samples_to_discard) {
				sensor.nb_samples_to_discard--;
				continue;
			}

			if (sensor.state.calib == LSM6DSV16BX_CALIBRATION_NOT_CALIBRATING)
			{
				_data_handler_recording(&f_data);
			} else if (sensor.state.calib == LSM6DSV16BX_CALIBRATION_RECORDING)
			{
				calibration_result = _data_handler_calibrating(&f_data, gbias_tmp);
				if (calibration_result)
				{
					break;
				}
			}
		}

		if (sensor.state.calib == LSM6DSV16BX_CALIBRATION_RECORDING && calibration_result)
		{
			k_timer_stop(&calibration_timer);
			lsm6dsv16bx_reset();
			if (sensor.callbacks.lsm6dsv16bx_calibration_result_cb)
			{
				(*sensor.callbacks.lsm6dsv16bx_calibration_result_cb)(calibration_result, gbias_tmp[0], gbias_tmp[1], gbias_tmp[2]);
			} else {
				LOG_ERR("No Calibration callback defined!");
			}
		}
	}

	if (sensor.state.int2_on_int1)
	{
		handled = true;
		lsm6dsv16bx_int2_irq(item);
	}

	if (!handled)
	{
		LOG_WRN("IMU interrupt 1 fired, but not handled!");
	}
}

static void _check_fsm_callbacks()
{
#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_1
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[0]) {
		LOG_ERR("No callback defined for algorithm %s", CONFIG_LSM6DSV16BX_FSM_ALG_1_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_2
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[1]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_2_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_3
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[2]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_3_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_4
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[3]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_4_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_5
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[4]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_5_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_6
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[5]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_6_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_7
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[6]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_7_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_8
	if (!sensor.callbacks.lsm6dsv16bx_fsm_cbs[7]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_8_NAME);
	}
#endif
}

static void _check_fsm_config(const ucf_line_t* cfgs[LSM6DSV16BX_FSM_ALG_MAX_NB])
{
#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_1
	if (!sensor.fsm_configs.fsm_ucf_cfg[0]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_1_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_2
	if (!sensor.fsm_configs.fsm_ucf_cfg[1]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_2_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_3
	if (!sensor.fsm_configs.fsm_ucf_cfg[2]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_3_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_4
	if (!sensor.fsm_configs.fsm_ucf_cfg[3]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_4_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_5
	if (!sensor.fsm_configs.fsm_ucf_cfg[4]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_5_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_6
	if (!sensor.fsm_configs.fsm_ucf_cfg[5]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_6_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_7
	if (!sensor.fsm_configs.fsm_ucf_cfg[6]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_7_NAME);
	}
#endif

#ifdef CONFIG_LSM6DSV16BX_FSM_USE_ALG_8
	if (!sensor.fsm_configs.fsm_ucf_cfg[7]) {
		LOG_ERR("No %s configuration defined", CONFIG_LSM6DSV16BX_FSM_ALG_8_NAME);
	}
#endif
}

static void lsm6dsv16bx_scale_init(lsm6dsv16bx_xl_full_scale_t xl, lsm6dsv16bx_gy_full_scale_t gy)
{
	sensor.scale.xl_scale = xl;
	switch (sensor.scale.xl_scale)
	{
	case LSM6DSV16BX_2g:
		sensor.scale.xl_conversion_function = lsm6dsv16bx_from_fs2_to_mg;
		break;
	case LSM6DSV16BX_4g:
		sensor.scale.xl_conversion_function = lsm6dsv16bx_from_fs4_to_mg;
		break;
	case LSM6DSV16BX_8g:
		sensor.scale.xl_conversion_function = lsm6dsv16bx_from_fs8_to_mg;
		break;
	case LSM6DSV16BX_16g:
		sensor.scale.xl_conversion_function = lsm6dsv16bx_from_fs16_to_mg;
		break;

	default:
		LOG_WRN("%u is not a valid xl scale. Setting to %u", xl, LSM6DSV16BX_4g);
		sensor.scale.xl_scale = LSM6DSV16BX_4g;
		sensor.scale.xl_conversion_function = lsm6dsv16bx_from_fs4_to_mg;
		break;
	}

	sensor.scale.gy_scale = gy;
	switch (sensor.scale.gy_scale)
	{
	case LSM6DSV16BX_125dps:
		sensor.scale.gy_conversion_function = lsm6dsv16bx_from_fs125_to_mdps;
		break;
	case LSM6DSV16BX_250dps:
		sensor.scale.gy_conversion_function = lsm6dsv16bx_from_fs250_to_mdps;
		break;
	case LSM6DSV16BX_500dps:
		sensor.scale.gy_conversion_function = lsm6dsv16bx_from_fs500_to_mdps;
		break;
	case LSM6DSV16BX_1000dps:
		sensor.scale.gy_conversion_function = lsm6dsv16bx_from_fs1000_to_mdps;
		break;
	case LSM6DSV16BX_2000dps:
		sensor.scale.gy_conversion_function = lsm6dsv16bx_from_fs2000_to_mdps;
		break;
	case LSM6DSV16BX_4000dps:
		sensor.scale.gy_conversion_function = lsm6dsv16bx_from_fs4000_to_mdps;
		break;

	default:
		LOG_WRN("%u is not a valid gyroscope scale. Setting to %u", gy, LSM6DSV16BX_2000dps);
		sensor.scale.gy_scale = LSM6DSV16BX_2000dps;
		sensor.scale.gy_conversion_function = lsm6dsv16bx_from_fs2000_to_mdps;
		break;
	}
}

void lsm6dsv16bx_init(lsm6dsv16bx_cb_t cb, lsm6dsv16bx_fsm_cfg_t fsm_cfg)
{
	sensor.callbacks = cb;
	if (!sensor.callbacks.lsm6dsv16bx_ts_sample_cb)
	{
		LOG_ERR("No Timestamp callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_acc_sample_cb)
	{
		LOG_ERR("No Accelerometer callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_gyro_sample_cb)
	{
		LOG_ERR("No Gyrometer callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_qvar_sample_cb)
	{
		LOG_ERR("No QVar callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_gbias_sample_cb)
	{
		LOG_ERR("No Gyroscope Bias callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_game_rot_sample_cb)
	{
		LOG_ERR("No Game Rotation callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_gravity_sample_cb)
	{
		LOG_ERR("No Gravity callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_calibration_result_cb)
	{
		LOG_ERR("No Calibration callback defined!");
	}

	if (!sensor.callbacks.lsm6dsv16bx_sigmot_cb)
	{
		LOG_ERR("No Significant Motion callback defined!");
	}

	_check_fsm_callbacks();

	sensor.fsm_configs = fsm_cfg;
	_check_fsm_config(sensor.fsm_configs.fsm_ucf_cfg);

	int res = attach_interrupt(imu_int_1, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE, &imu_int_1_cb_data, imu_int_1_cb);
	if (res != 0) {
		LOG_ERR("Error while attaching interrupt 1 %i", res);
	}

	k_work_init(&imu_int1_work, lsm6dsv16bx_int1_irq);

#if DT_NODE_EXISTS(imu_int2)
	res = attach_interrupt(imu_int_2, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE, &imu_int_2_cb_data, imu_int_2_cb);
	if (res != 0) {
		LOG_ERR("Error while attaching interrupt 2 %i", res);
	}

	k_work_init(&imu_int2_work, lsm6dsv16bx_int2_irq);
#endif

	lsm6dsv16bx_reset_t rst;

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
	lsm6dsv16bx_device_id_get(&sensor.dev_ctx, &sensor.whoamI);

	if (sensor.whoamI != LSM6DSV16BX_ID) {
		LOG_ERR("Error while fetching WHO_AM_I register. Received %#02x, expected %#02x", sensor.whoamI, LSM6DSV16BX_ID);
		return;
	}

	/* Restore default configuration */
  	lsm6dsv16bx_reset_set(&sensor.dev_ctx, LSM6DSV16BX_GLOBAL_RST);
	do {
		lsm6dsv16bx_reset_get(&sensor.dev_ctx, &rst);
	} while (rst != LSM6DSV16BX_READY);

	lsm6dsv16bx_scale_init(LSM6DSV16BX_4g, LSM6DSV16BX_2000dps);

	lsm6dsv16bx_state_t tmp_state = {
		.xl_enabled = false,
		.gy_enabled = false,
		.sflp_state = {
			.gbias_enabled = false,
			.game_rot_enabled = false,
			.gravity_enabled = false,
		},
		.sigmot_enabled = false,
		.fsm_enabled = false,
		.calib = LSM6DSV16BX_CALIBRATION_NOT_CALIBRATING,
		.int2_on_int1 = false,
	};
	memcpy(&sensor.state, &tmp_state, sizeof(tmp_state));
}
