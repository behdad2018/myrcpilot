/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: trapz.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Sep-2020 23:59:27
 */

/* Include Files */
#include "trapz.h"
#include "thrust.h"
#include <arm_neon.h>
#include <string.h>

/* Function Definitions */

/*
 * Arguments    : const float x[36]
 *                const float y[36]
 * Return Type  : float
 */
float b_trapz(const float x[36], const float y[36])
{
  float z;
  float c[36];
  int ix;
  int iac;
  int b_iac;
  int ia;
  c[0] = 0.5F * (x[1] - x[0]);
  for (ix = 0; ix <= 28; ix += 4) {
    vst1q_f32(&c[ix + 1], vmulq_f32(vdupq_n_f32(0.5F), vsubq_f32(vld1q_f32(&x[ix
      + 2]), vld1q_f32(&x[ix]))));
  }

  c[33] = 0.5F * (x[34] - x[32]);
  c[34] = 0.5F * (x[35] - x[33]);
  c[35] = 0.5F * (x[35] - x[34]);
  z = 0.0F;
  ix = 0;
  for (iac = 0; iac < 36; iac++) {
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      z += y[ia - 1] * c[ix];
    }

    ix++;
  }

  return z;
}

/*
 * Arguments    : const float x[60]
 *                const float y[2160]
 *                float z[36]
 * Return Type  : void
 */
void trapz(const float x[60], const float y[2160], float z[36])
{
  float c[60];
  int ix;
  int iac;
  int iy;
  int i;
  int ia;
  c[0] = 0.5F * (x[1] - x[0]);
  for (ix = 0; ix <= 52; ix += 4) {
    vst1q_f32(&c[ix + 1], vmulq_f32(vdupq_n_f32(0.5F), vsubq_f32(vld1q_f32(&x[ix
      + 2]), vld1q_f32(&x[ix]))));
  }

  c[57] = 0.5F * (x[58] - x[56]);
  c[58] = 0.5F * (x[59] - x[57]);
  c[59] = 0.5F * (x[59] - x[58]);
  memset(&z[0], 0, 36U * sizeof(float));
  ix = 0;
  for (iac = 0; iac <= 2124; iac += 36) {
    iy = 0;
    i = iac + 36;
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
