#pragma once

#include <zephyr/drivers/spi.h>

#define BOOT_TIME 10 //ms

void lsm6dsv16x_init();
void lsm6dsv16x_irq();
