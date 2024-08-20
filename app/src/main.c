/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <usb_mass_storage/usb_mass_storage.h>

#include <app_version.h>


LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
	int ret;

	printk("Zephyr Example Application %s\n", APP_VERSION_STRING);

	ret = xiao_ble_usb_mass_storage_init();

	if (ret) {
		LOG_ERR("The device could not be put in USB mass storage mode.");
		return 0;
	}

	LOG_INF("The device is put in USB mass storage mode.\n");
	return 0;
}
