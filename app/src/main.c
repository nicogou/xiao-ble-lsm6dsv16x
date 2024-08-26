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

struct line {
	float_t acc_x;
	float_t acc_y;
	float_t acc_z;
	bool acc_updated;
	float_t gyro_x;
	float_t gyro_y;
	float_t gyro_z;
	bool gyro_updated;
	int32_t ts_start;
	int32_t ts;
	bool ts_updated;
	uint8_t nb_samples_to_discard;
};

static struct line l = {
	.acc_updated = false,
	.gyro_updated = false,
	.ts_updated = false,
	.ts_start = -1,
	.nb_samples_to_discard = 5,
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

		sprintf(txt, "%i,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n", l.ts, (double)l.acc_x, (double)l.acc_y, (double)l.acc_z, (double)l.gyro_x, (double)l.gyro_y, (double)l.gyro_z);
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

static void ts_received_cb(int ts)
{
	if (l.ts_start == -1) {
		l.ts_start = ts;
	}
	l.ts = (ts - l.ts_start) / 768;
	l.ts_updated = true;

	print_line_if_needed();
	return;
}

int main(void)
{
	int ret;

	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

	lsm6dsv16x_cb_t callbacks = {
		.lsm6dsv16x_ts_sample_cb = ts_received_cb,
		.lsm6dsv16x_acc_sample_cb = acc_received_cb,
		.lsm6dsv16x_gyro_sample_cb = gyro_received_cb
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
