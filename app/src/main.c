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

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
	int ret;

	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

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
