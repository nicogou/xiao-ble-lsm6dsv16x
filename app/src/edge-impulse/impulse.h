#pragma once

#include <zephyr/kernel.h>

int impulse_add_data(float* data, size_t len);
int impulse_start_predicting();
int impulse_stop_predicting();
int impulse_init();
