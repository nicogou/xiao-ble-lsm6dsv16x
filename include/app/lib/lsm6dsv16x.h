#pragma once

#include <zephyr/drivers/spi.h>
#include "lsm6dsv16x_reg.h"

/* Define the structure needed for FSM/MLC */
#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** Common data block definition **/
typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

#endif /* MEMS_UCF_SHARED_TYPES */

#define BOOT_TIME 10 //ms
#define FIFO_WATERMARK 200

typedef enum {
	LSM6DSV16X_OFF,
	LSM6DSV16X_IDLE,
	LSM6DSV16X_RECORDING,
	LSM6DSV16X_CALIBRATION_SETTLING,
	LSM6DSV16X_CALIBRATION_RECORDING,
	LSM6DSV16X_SIGNIFICANT_MOTION,
	LSM6DSV16X_FSM_LONG_TOUCH,
} lsm6dsv16x_state_t;

typedef struct {
	void (*lsm6dsv16x_ts_sample_cb)(float_t);
	void (*lsm6dsv16x_acc_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_gyro_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_gbias_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_game_rot_sample_cb)(float_t, float_t, float_t, float_t);
	void (*lsm6dsv16x_gravity_sample_cb)(float_t, float_t, float_t);
	void (*lsm6dsv16x_calibration_result_cb)(int, float_t, float_t, float_t);
	void (*lsm6dsv16x_sigmot_cb)();
	void (*lsm6dsv16x_fsm_alg_1_cb)(uint8_t);
} lsm6dsv16x_cb_t;

typedef struct {
	const ucf_line_t* fsm_alg_1;
} lsm6dsv16x_fsm_configs_t;

typedef struct {
	stmdev_ctx_t dev_ctx;
	uint8_t whoamI;
	lsm6dsv16x_cb_t callbacks;
	lsm6dsv16x_state_t state;
	uint8_t nb_samples_to_discard;
	lsm6dsv16x_fsm_configs_t fsm_configs;
} lsm6dsv16x_sensor_t;

void lsm6dsv16x_init(lsm6dsv16x_cb_t cb, lsm6dsv16x_fsm_configs_t cfgs);
void lsm6dsv16x_int1_irq(struct k_work *item);
int lsm6dsv16x_start_acquisition(bool enable_gbias, bool enable_sflp, bool enable_qvar);
int lsm6dsv16x_stop_acquisition();
int lsm6dsv16x_start_calibration();
int lsm6dsv16x_stop_calibration();
int lsm6dsv16x_start_significant_motion_detection();
int lsm6dsv16x_stop_significant_motion_detection();
int lsm6dsv16x_start_fsm_long_touch();
int lsm6dsv16x_stop_fsm_long_touch();
void lsm6dsv16x_set_gbias(float x, float y, float z);
