/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: circl_traj.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 14-Dec-2020 19:37:44
 */

#ifndef CIRCL_TRAJ_H
#define CIRCL_TRAJ_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"

/* Function Declarations */
extern void circl_traj(float x0, float b_y0, float z0, float counter, float R,
  float Vcruise, float nrev, float t_takeoff, float t_delay, float z_target,
  float f, float *x, float *y, float *z);

#endif

/*
 * File trailer for circl_traj.h
 *
 * [EOF]
 */
