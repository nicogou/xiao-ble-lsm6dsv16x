#pragma once

#include <zephyr/kernel.h>

#define ADAFRUIT_BOOTLOADER_UF2 0x57
#define ADAFRUIT_BOOTLOADER_OTA 0xA8
#define ADAFRUIT_BOOTLOADER_SERIAL 0x4E

typedef struct {
	bool (*adafruit_bootloader_uf2_check)();
	bool (*adafruit_bootloader_ota_check)();
	bool (*adafruit_bootloader_serial_check)();
} xiao_ble_shell_cd_t;

void xiao_ble_shell_init(xiao_ble_shell_cd_t cb);
