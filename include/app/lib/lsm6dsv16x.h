#pragma once

#include <zephyr/drivers/spi.h>
#include "lsm6dsv16x_reg.h"

#define BOOT_TIME 10 //ms
#define FIFO_WATERMARK 64

typedef struct {
	void (*lsm6dsv16x_ts_sample_cb)(float_t);
	void (*lsm6dsv16x_acc_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_gyro_sample_cb)(float_t, float_t, float_t);
} lsm6dsv16x_cb_t;

typedef struct {
	stmdev_ctx_t dev_ctx;
	uint8_t whoamI;
	lsm6dsv16x_cb_t callbacks;
} lsm6dsv16x_sensor_t;

void lsm6dsv16x_init(lsm6dsv16x_cb_t cb);
void lsm6dsv16x_irq(struct k_work *item);
int lsm6dsv16x_start_acquisition();
int lsm6dsv16x_stop_acquisition();
