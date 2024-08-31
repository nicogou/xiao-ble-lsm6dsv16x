/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if CONFIG_USB_MASS_STORAGE
#include <usb_mass_storage/usb_mass_storage.h>
#endif
#include <state_machine/state_machine.h>

#include <app_version.h>

#include <app/lib/lsm6dsv16x.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static struct fs_file_t calibration_file;

struct line {
	float_t acc_x;
	float_t acc_y;
	float_t acc_z;
	bool acc_updated;
	float_t gyro_x;
	float_t gyro_y;
	float_t gyro_z;
	bool gyro_updated;
	float_t ts;
	bool ts_updated;
	uint8_t nb_samples_to_discard;
	float_t gbias_x;
	float_t gbias_y;
	float_t gbias_z;
	bool gbias_updated;
	float_t gravity_x;
	float_t gravity_y;
	float_t gravity_z;
	bool gravity_updated;
	float_t game_rot_x;
	float_t game_rot_y;
	float_t game_rot_z;
	float_t game_rot_w;
	bool game_rot_updated;
};

static struct line l = {
	.acc_updated = false,
	.gyro_updated = false,
	.ts_updated = false,
	.nb_samples_to_discard = 5,
	.gbias_updated = false,
	.gravity_updated = false,
	.game_rot_updated = false,
};

static void print_line_if_needed(){
	char txt[100];
	if (l.acc_updated && l.gyro_updated && l.ts_updated) {
		l.acc_updated = false;
		l.gyro_updated = false;
		l.ts_updated = false;

		if (l.nb_samples_to_discard) {
			l.nb_samples_to_discard--;
			return;
		}

		sprintf(txt, "%.3f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n", (double)l.ts, (double)l.acc_x, (double)l.acc_y, (double)l.acc_z, (double)l.gyro_x, (double)l.gyro_y, (double)l.gyro_z);
		int res = usb_mass_storage_write_to_current_session(txt, strlen(txt));

		if (res < 0) {
			LOG_ERR("Unable to write to session file, ending session");
			state_machine_post_event(XIAO_EVENT_STOP_RECORDING);
		}
	}
}

static void acc_received_cb(float_t x, float_t y, float_t z)
{
	l.acc_x = x;
	l.acc_y = y;
	l.acc_z = z;
	l.acc_updated = true;

	print_line_if_needed();
	return;
}

static void gyro_received_cb(float_t x, float_t y, float_t z)
{
	l.gyro_x = x;
	l.gyro_y = y;
	l.gyro_z = z;
	l.gyro_updated = true;

	print_line_if_needed();
	return;
}

static void ts_received_cb(float_t ts)
{
	l.ts = ts / 1000000; // Convert ns to ms.
	l.ts_updated = true;

	print_line_if_needed();
	return;
}

static void gbias_received_cb(float_t x, float_t y, float_t z)
{
	l.gbias_x = x;
	l.gbias_y = y;
	l.gbias_z = z;
	l.gbias_updated = true;
	LOG_DBG("Received gyroscope bias: x=%f, y=%f, z=%f", (double)x, (double)y, (double)z);
}

static void gravity_received_cb(float_t x, float_t y, float_t z)
{
	l.gravity_x = x;
	l.gravity_y = y;
	l.gravity_z = z;
	l.gravity_updated = true;
	LOG_DBG("Received gravity: x=%f, y=%f, z=%f", (double)x, (double)y, (double)z);
}

static void game_rot_received_cb(float_t x, float_t y, float_t z, float_t w)
{
	l.game_rot_x = x;
	l.game_rot_y = y;
	l.game_rot_z = z;
	l.game_rot_w = w;
	l.game_rot_updated = true;
	LOG_DBG("Received game rotation: x=%f, y=%f, z=%f, w=%f", (double)x, (double)y, (double)z, (double)w);
}

static void calib_res_cb(float_t x, float_t y, float_t z)
{
	LOG_INF("Calibration succeeded. Gbias x: %f y: %f z: %f", (double)x, (double)y, (double)z);
	int res = usb_mass_storage_create_file(NULL, "cal.txt", &calibration_file, true);
	if (res != 0)
	{
		LOG_ERR("Error creating calibration file (%i)", res);
	} else {
		char txt[50];
		sprintf(txt, "x:%.2f\ny:%.2f\nz:%.2f", (double)x, (double)y, (double)z);
		res = usb_mass_storage_write_to_file(txt, strlen(txt), &calibration_file, true);
		if (res)
		{
			LOG_ERR("failed to write to cal file");
		}
		res = usb_mass_storage_close_file(&calibration_file);
		if (res)
		{
			LOG_ERR("Failed to close cal file");
		}
	}

	state_machine_post_event(XIAO_EVENT_STOP_CALIBRATION);
}

int main(void)
{
	int ret;

	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

	lsm6dsv16x_cb_t callbacks = {
		.lsm6dsv16x_ts_sample_cb = ts_received_cb,
		.lsm6dsv16x_acc_sample_cb = acc_received_cb,
		.lsm6dsv16x_gyro_sample_cb = gyro_received_cb,
		.lsm6dsv16x_gbias_sample_cb = gbias_received_cb,
		.lsm6dsv16x_gravity_sample_cb = gravity_received_cb,
		.lsm6dsv16x_game_rot_sample_cb = game_rot_received_cb,
		.lsm6dsv16x_calibration_result_cb = calib_res_cb,
	};

	lsm6dsv16x_init(callbacks);

#if CONFIG_USB_MASS_STORAGE
	ret = usb_mass_storage_init();

	if (ret) {
		LOG_ERR("The device could not be put in USB mass storage mode.");
		return 0;
	}

	LOG_INF("The device is put in USB mass storage mode.");
#endif

	state_machine_init();

	return state_machine_run();
}
