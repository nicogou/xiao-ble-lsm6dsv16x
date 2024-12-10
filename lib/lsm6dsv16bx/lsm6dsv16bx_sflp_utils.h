#include <zephyr/kernel.h>
#include "app/lib/lsm6dsv16bx.h"

uint32_t npy_halfbits_to_floatbits(uint16_t h);
float_t npy_half_to_float(uint16_t h);
void sflp2q(float quat[4], uint16_t sflp[3]);
