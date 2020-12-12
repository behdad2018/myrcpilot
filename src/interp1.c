/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: interp1.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 11-Dec-2020 01:16:32
 */

/* Include Files */
#include "interp1.h"
#include "rpmtothrottle.h"
#include <string.h>

/* Function Definitions */

/*
 * Arguments    : const float varargin_1[4]
 *                const float varargin_2[40]
 *                float varargin_3
 *                float Vq[10]
 * Return Type  : void
 */
void interp1(const float varargin_1[4], const float varargin_2[40], float
             varargin_3, float Vq[10])
{
  float y[40];
  float x[4];
  int offset;
  int low_i;
  int high_i;
  float tmp;
  int mid_i;
  float b_y1;
  memcpy(&y[0], &varargin_2[0], 40U * sizeof(float));
  x[0] = varargin_1[0];
  x[1] = varargin_1[1];
  x[2] = varargin_1[2];
  x[3] = varargin_1[3];
  if (varargin_1[1] < varargin_1[0]) {
    x[0] = varargin_1[3];
    x[3] = varargin_1[0];
    x[1] = varargin_1[2];
    x[2] = varargin_1[1];
    for (high_i = 0; high_i < 10; high_i++) {
      offset = (high_i << 2) + 4;
      tmp = y[offset + -4];
      y[offset + -4] = y[offset - 1];
      y[offset - 1] = tmp;
      tmp = y[offset + -3];
      y[offset + -3] = y[offset - 2];
      y[offset - 2] = tmp;
    }
  }

  for (offset = 0; offset < 10; offset++) {
    Vq[offset] = 0.0F;
  }

  if ((varargin_3 <= x[3]) && (varargin_3 >= x[0])) {
    low_i = 1;
    offset = 2;
    high_i = 4;
    while (high_i > offset) {
      mid_i = (low_i + high_i) >> 1;
      if (varargin_3 >= x[mid_i - 1]) {
        low_i = mid_i;
        offset = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    tmp = x[low_i - 1];
    tmp = (varargin_3 - tmp) / (x[low_i] - tmp);
    if (tmp == 0.0F) {
      for (high_i = 0; high_i < 10; high_i++) {
        Vq[high_i] = y[(low_i + (high_i << 2)) - 1];
      }
    } else if (tmp == 1.0F) {
      for (high_i = 0; high_i < 10; high_i++) {
        Vq[high_i] = y[low_i + (high_i << 2)];
      }
    } else {
      for (high_i = 0; high_i < 10; high_i++) {
        offset = low_i + (high_i << 2);
        b_y1 = y[offset - 1];
        if (b_y1 == y[offset]) {
          Vq[high_i] = b_y1;
        } else {
          Vq[high_i] = (1.0F - tmp) * b_y1 + tmp * y[offset];
        }
      }
    }
  }
}

/*
 * File trailer for interp1.c
 *
 * [EOF]
 */
