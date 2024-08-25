#pragma once

#include <zephyr/drivers/spi.h>
#include "lsm6dsv16x_reg.h"

#define BOOT_TIME 10 //ms
#define FIFO_WATERMARK 64

typedef struct {
	stmdev_ctx_t dev_ctx;
	uint8_t whoamI;
	void (*lsm6dsv16x_fifo_cb)();
} lsm6dsv16x_sensor_t;

void lsm6dsv16x_init(void (*fifo_cb)());
void lsm6dsv16x_irq(struct k_work *item);
int lsm6dsv16x_start_acquisition();
int lsm6dsv16x_stop_acquisition();
