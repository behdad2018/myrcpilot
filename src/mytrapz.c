/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mytrapz.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 04-Jan-2021 16:06:00
 */

/* Include Files */
#include "mytrapz.h"
#include "thrust.h"

/* Function Definitions */

/*
 * TRAPZ  Trapezoidal numerical integration.
 *    Z = TRAPZ(Y) computes an approximation of the integral of Y via
 *    the trapezoidal method (with unit spacing).  To compute the integral
 *    for spacing different from one, multiply Z by the spacing increment.
 *
 *    For vectors, TRAPZ(Y) is the integral of Y. For matrices, TRAPZ(Y)
 *    is a row vector with the integral over each column. For N-D
 *    arrays, TRAPZ(Y) works across the first non-singleton dimension.
 *
 *    Z = TRAPZ(X,Y) computes the integral of Y with respect to X using the
 *    trapezoidal method. X can be a scalar or a vector with the same length
 *    as the first non-singleton dimension in Y. TRAPZ operates along this
 *    dimension. If X is scalar, then TRAPZ(X,Y) is equivalent to X*TRAPZ(Y).
 *
 *    Z = TRAPZ(X,Y,DIM) or TRAPZ(Y,DIM) integrates across dimension DIM
 *    of Y. The length of X must be the same as size(Y,DIM)).
 *
 *    Example:
 *        Y = [0 1 2; 3 4 5]
 *        trapz(Y,1)
 *        trapz(Y,2)
 *
 *    Class support for inputs X, Y:
 *       float: double, single
 *
 *    See also SUM, CUMSUM, CUMTRAPZ, INTEGRAL.
 * Arguments    : const float x[13]
 *                const float y[13]
 * Return Type  : float
 */
float b_mytrapz(const float x[13], const float y[13])
{
  float z;
  int ixLead;
  int iyLead;
  float work;
  int m;
  float tmp2;
  float b_y1[12];

  /*    Copyright 1984-2017 The MathWorks, Inc. */
  /*    Make sure x and y are column vectors, or y is a matrix. */
  /* [y,nshifts] = shiftdim(y) */
  /*  Trapezoid sum computed with vector-matrix multiply. */
  ixLead = 1;
  iyLead = 0;
  work = x[0];
  for (m = 0; m < 12; m++) {
    tmp2 = work;
    work = x[ixLead];
    b_y1[iyLead] = x[ixLead] - tmp2;
    ixLead++;
    iyLead++;
  }

  work = 0.0F;
  for (ixLead = 0; ixLead < 12; ixLead++) {
    work += b_y1[ixLead] * (y[ixLead] + y[ixLead + 1]);
  }

  z = work * 0.5F;

  /*  siz = size(y); */
  /*  siz(1) = 1; */
  /*  z = reshape(z,[ones(1,nshifts),siz]); */
  /*  if ~isempty(perm) && ~isscalar(z) */
  /*      z = ipermute(z,perm); */
  /*  end */
  return z;
}

/*
 * TRAPZ  Trapezoidal numerical integration.
 *    Z = TRAPZ(Y) computes an approximation of the integral of Y via
 *    the trapezoidal method (with unit spacing).  To compute the integral
 *    for spacing different from one, multiply Z by the spacing increment.
 *
 *    For vectors, TRAPZ(Y) is the integral of Y. For matrices, TRAPZ(Y)
 *    is a row vector with the integral over each column. For N-D
 *    arrays, TRAPZ(Y) works across the first non-singleton dimension.
 *
 *    Z = TRAPZ(X,Y) computes the integral of Y with respect to X using the
 *    trapezoidal method. X can be a scalar or a vector with the same length
 *    as the first non-singleton dimension in Y. TRAPZ operates along this
 *    dimension. If X is scalar, then TRAPZ(X,Y) is equivalent to X*TRAPZ(Y).
 *
 *    Z = TRAPZ(X,Y,DIM) or TRAPZ(Y,DIM) integrates across dimension DIM
 *    of Y. The length of X must be the same as size(Y,DIM)).
 *
 *    Example:
 *        Y = [0 1 2; 3 4 5]
 *        trapz(Y,1)
 *        trapz(Y,2)
 *
 *    Class support for inputs X, Y:
 *       float: double, single
 *
 *    See also SUM, CUMSUM, CUMTRAPZ, INTEGRAL.
 * Arguments    : const float x[18]
 *                const float y[234]
 *                float z[13]
 * Return Type  : void
 */
void mytrapz(const float x[18], const float y[234], float z[13])
{
  int ixLead;
  int iyLead;
  float work;
  float b_y1[234];
  int m;
  float tmp2;
  float c_y1[17];

  /*    Copyright 1984-2017 The MathWorks, Inc. */
  /*    Make sure x and y are column vectors, or y is a matrix. */
  for (ixLead = 0; ixLead < 13; ixLead++) {
    for (iyLead = 0; iyLead < 18; iyLead++) {
      b_y1[iyLead + 18 * ixLead] = y[ixLead + 13 * iyLead];
    }
  }

  ixLead = 1;
  iyLead = 0;
  work = x[0];
  for (m = 0; m < 17; m++) {
    tmp2 = work;
    work = x[ixLead];
    c_y1[iyLead] = x[ixLead] - tmp2;
    ixLead++;
    iyLead++;
  }

  for (ixLead = 0; ixLead < 13; ixLead++) {
    work = 0.0F;
    for (iyLead = 0; iyLead < 17; iyLead++) {
      m = iyLead + 18 * ixLead;
      work += c_y1[iyLead] * (b_y1[m] + b_y1[m + 1]);
    }

    z[ixLead] = work * 0.5F;
  }

  /*  siz = size(y); */
  /*  siz(1) = 1; */
  /*  z = reshape(z,[ones(1,nshifts),siz]); */
  /*  if ~isempty(perm) && ~isscalar(z) */
  /*      z = ipermute(z,perm); */
  /*  end */
}

/*
 * File trailer for mytrapz.c
 *
 * [EOF]
 */
