#pragma once

#include <zephyr/drivers/spi.h>
#include "lsm6dsv16x_reg.h"

#define BOOT_TIME 10 //ms
#define FIFO_WATERMARK 200

typedef enum {
	LSM6DSV16X_OFF,
	LSM6DSV16X_IDLE,
	LSM6DSV16X_RECORDING,
	LSM6DSV16X_CALIBRATION_SETTLING,
	LSM6DSV16X_CALIBRATION_RECORDING,
} lsm6dsv16x_state_t;

typedef struct {
	void (*lsm6dsv16x_ts_sample_cb)(float_t);
	void (*lsm6dsv16x_acc_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_gyro_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_gbias_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_game_rot_sample_cb)(float_t, float_t, float_t, float_t);
	void (*lsm6dsv16x_gravity_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_calibration_result_cb)(float_t, float_t, float_t);
} lsm6dsv16x_cb_t;

typedef struct {
	stmdev_ctx_t dev_ctx;
	uint8_t whoamI;
	lsm6dsv16x_cb_t callbacks;
	lsm6dsv16x_state_t state;
	uint8_t nb_samples_to_discard;
} lsm6dsv16x_sensor_t;

void lsm6dsv16x_init(lsm6dsv16x_cb_t cb);
void lsm6dsv16x_irq(struct k_work *item);
int lsm6dsv16x_start_acquisition(bool enable_gbias, bool enable_sflp);
int lsm6dsv16x_stop_acquisition();
int lsm6dsv16x_start_calibration();
int lsm6dsv16x_stop_calibration();
void lsm6dsv16x_set_gbias(float x, float y, float z);
