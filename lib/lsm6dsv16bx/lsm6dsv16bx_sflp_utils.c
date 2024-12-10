#include "lsm6dsv16bx_sflp_utils.h"
#include <math.h>
#include "app/lib/lsm6dsv16bx.h"

/*
 * Original conversion routines taken from: https://github.com/numpy/numpy
 *
 * uint32_t npy_halfbits_to_floatbits(uint16_t h);
 * float_t npy_half_to_float(uint16_t h);
 *
 * Released under BSD-3-Clause License
 */
uint32_t npy_halfbits_to_floatbits(uint16_t h)
{
    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    h_exp = (h&0x7c00u);
    f_sgn = ((uint32_t)h&0x8000u) << 16;
    switch (h_exp) {
        case 0x0000u: /* 0 or subnormal */
            h_sig = (h&0x03ffu);
            /* Signed zero */
            if (h_sig == 0) {
                return f_sgn;
            }
            /* Subnormal */
            h_sig <<= 1;
            while ((h_sig&0x0400u) == 0) {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32_t)(h_sig&0x03ffu)) << 13;
            return f_sgn + f_exp + f_sig;
        case 0x7c00u: /* inf or NaN */
            /* All-ones exponent and a copy of the significand */
            return f_sgn + 0x7f800000u + (((uint32_t)(h&0x03ffu)) << 13);
        default: /* normalized */
            /* Just need to adjust the exponent and shift */
            return f_sgn + (((uint32_t)(h&0x7fffu) + 0x1c000u) << 13);
    }
}

float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = npy_halfbits_to_floatbits(h);
    return conv.ret;
}

void sflp2q(float quat[4], uint16_t sflp[3])
{
  float sumsq = 0;

  quat[0] = npy_half_to_float(sflp[0]);
  quat[1] = npy_half_to_float(sflp[1]);
  quat[2] = npy_half_to_float(sflp[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f) {
    float n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
}
