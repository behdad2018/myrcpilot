/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: trapz.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 02-Jan-2021 07:00:08
 */

/* Include Files */
#include "trapz.h"
#include "thrust.h"
#include <arm_neon.h>

/* Function Definitions */

/*
 * Arguments    : const float x[13]
 *                const float y[13]
 * Return Type  : float
 */
float b_trapz(const float x[13], const float y[13])
{
  float z;
  float c[13];
  float32x4_t r;
  int ix;
  int iac;
  int b_iac;
  int ia;
  c[0] = 0.5F * (x[1] - x[0]);
  r = vdupq_n_f32(0.5F);
  vst1q_f32(&c[1], vmulq_f32(r, vsubq_f32(vld1q_f32(&x[2]), vld1q_f32(&x[0]))));
  vst1q_f32(&c[5], vmulq_f32(r, vsubq_f32(vld1q_f32(&x[6]), vld1q_f32(&x[4]))));
  c[9] = 0.5F * (x[10] - x[8]);
  c[10] = 0.5F * (x[11] - x[9]);
  c[11] = 0.5F * (x[12] - x[10]);
  c[12] = 0.5F * (x[12] - x[11]);
  z = 0.0F;
  ix = 0;
  for (iac = 0; iac < 13; iac++) {
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      z += y[ia - 1] * c[ix];
    }

    ix++;
  }

  return z;
}

/*
 * Arguments    : const float x[18]
 *                const float y[234]
 *                float z[13]
 * Return Type  : void
 */
void trapz(const float x[18], const float y[234], float z[13])
{
  float c[18];
  float32x4_t r;
  int iy;
  int ix;
  int iac;
  int i;
  int ia;
  c[0] = 0.5F * (x[1] - x[0]);
  r = vdupq_n_f32(0.5F);
  vst1q_f32(&c[1], vmulq_f32(r, vsubq_f32(vld1q_f32(&x[2]), vld1q_f32(&x[0]))));
  vst1q_f32(&c[5], vmulq_f32(r, vsubq_f32(vld1q_f32(&x[6]), vld1q_f32(&x[4]))));
  vst1q_f32(&c[9], vmulq_f32(r, vsubq_f32(vld1q_f32(&x[10]), vld1q_f32(&x[8]))));
  vst1q_f32(&c[13], vmulq_f32(r, vsubq_f32(vld1q_f32(&x[14]), vld1q_f32(&x[12]))));
  c[17] = 0.5F * (x[17] - x[16]);
  for (iy = 0; iy < 13; iy++) {
    z[iy] = 0.0F;
  }

  ix = 0;
  for (iac = 0; iac <= 221; iac += 13) {
    iy = 0;
    i = iac + 13;
    for (ia = iac + 1; ia <= i; ia++) {
      z[iy] += y[ia - 1] * c[ix];
      iy++;
    }

    ix++;
  }
}

/*
 * File trailer for trapz.c
 *
 * [EOF]
 */
