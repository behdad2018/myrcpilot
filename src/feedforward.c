/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: feedforward.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 05-Dec-2020 18:33:54
 */

/* Include Files */
#include "feedforward.h"
#include <math.h>


// rpm will cancell out, so it does not matter what the value is. Let's say 3000
const float rpm=3000;

/* Function Declarations */
static float __anon_fcn(const float V_rel_B[3], float rho, float T);

/* Function Definitions */

/*
 * Arguments    : float rpm
 *                const float V_rel_B[3]
 *                float rho
 *                float T
 * Return Type  : float
 */
static float __anon_fcn(const float V_rel_B[3], float rho, float T)
{
  float out_tmp;
  float a_tmp;
  float a;
  float b_a;
  float alpha;
  float cdrag_tmp_tmp;
  float cdrag_tmp;
  float Dp;

  /*  rotor characteristics */
  /*  propeller radius  [m] */
  /*  number of blade */
  /*  quad-copter weight */
  /*  blade characteristics */
  /*  blade solidity */
  /*  */
  /*  relative incoming velocity */
  /*  2*pi/60 = 0.1047197 */
  /*  rad per second */

  out_tmp = rpm * 0.104719698F * 0.1016F;

  /*  tip velocity */
  a_tmp = V_rel_B[0] * V_rel_B[0] + V_rel_B[1] * V_rel_B[1];
  a = sqrtf(a_tmp) / (out_tmp + 0.0001F);

  /*  advance ratio */
  b_a = V_rel_B[2] / (out_tmp + 0.0001F);

  /*  total external inflow ratio (effect lamb_c ans mu * tan alpha) */
  /*  the pitch angle of the quad-copter is already embded becuse the Vinf is */
  /*  in body frame! */
  a_tmp += V_rel_B[2] * V_rel_B[2];
  alpha = atanf(V_rel_B[2] / (sqrtf(a_tmp) + 1.0E-5F));

  /*  copter angle of attack */
  /*  blade profile drag */
  /*  Cd0=0.0081-0.0216*aoa+0.4*aoa^2; from helicopter literature, note aoa is the angle of attack - page 14, chapter 2 of Friedmann notes */
  /*  drag due to a quad-copter rotor  */
  /*  thrust coeffcient for a rotor */
  cdrag_tmp_tmp = fabsf(alpha);
  cdrag_tmp = sinf(cdrag_tmp_tmp);

  /*  this is in body frame */
  Dp = 0.5F * rho * 0.024F * a_tmp * 0.5F;
  a_tmp = rho * 3.14159274F * 0.0103225596F * ((out_tmp + 0.0001F) * (out_tmp +
    0.0001F));
  a = Dp * cosf(alpha) + 4.0F * ((0.2601F * (0.25F * T / a_tmp) * (cdrag_tmp +
    0.4837F) + 187.952194F * (a * a + b_a * b_a) * 0.008F * 0.108383402F *
    (cdrag_tmp + -0.1391F)) * cosf(cdrag_tmp_tmp)) * a_tmp;
  return (T - Dp * sinf(alpha)) - sqrtf(113.54229F - a * a);
}

/*
 * rotor characteristics
 * Arguments    : float rpm
 *                const float V_rel_B[3]
 *                float rho
 *                float *T_ref
 *                float *phi_ref
 *                float *theta_ref
 * Return Type  : void
 */
void feedforward(const float V_rel_B[3], float rho, float *T_ref,
                 float *phi_ref, float *theta_ref)
{
  float out;
  float fa;
  float a_tmp;
  float a;
  float b_a;
  float alpha_tmp;
  float alpha;
  float c_a;
  float fb;
  float fc;
  float c;
  float e;
  float p;
  float d;
  bool exitg1;
  float m;
  float s;
  float r;

  /*      Copyright (C) 12/01/2020  Regents of the University of Michigan */
  /*      Aerospace Engineering Department */
  /*      Computational Aeroscience Lab, written by Behdad Davoudi */
  /*      This program is free software: you can redistribute it and/or modify */
  /*      it under the terms of the GNU General Public License as published by */
  /*      the Free Software Foundation, either version 3 of the License, or */
  /*      (at your option) any later version. */
  /*   */
  /*      This program is distributed in the hope that it will be useful, */
  /*      but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*      GNU General Public License for more details. */
  /*   */
  /*      You should have received a copy of the GNU General Public License */
  /*      along with this program.  If not, see <https://www.gnu.org/licenses/>. */
  /*  propeller radius  [m] */
  /*  number of blade */
  /*  quad-copter weight */
  /*  blade characteristics */
  /*  blade solidity */
  /*  */
  /*  relative incoming velocity */
  /*  2*pi/60 = 0.1047197 */
  /*  rad per second */
  out = rpm * 0.104719698F * 0.1016F;

  /*  tip velocity */
  fa = V_rel_B[0] * V_rel_B[0] + V_rel_B[1] * V_rel_B[1];
  a_tmp = sqrtf(fa);
  a = a_tmp / (out + 0.0001F);

  /*  advance ratio */
  b_a = V_rel_B[2] / (out + 0.0001F);

  /*  total external inflow ratio (effect lamb_c ans mu * tan alpha) */
  /*  the pitch angle of the quad-copter is already embded becuse the Vinf is */
  /*  in body frame! */
  alpha_tmp = fa + V_rel_B[2] * V_rel_B[2];
  alpha = atanf(V_rel_B[2] / (sqrtf(alpha_tmp) + 1.0E-5F));

  /*  copter angle of attack */
  /*  blade profile drag */
  /*  Cd0=0.0081-0.0216*aoa+0.4*aoa^2; from helicopter literature, note aoa is the angle of attack - page 14, chapter 2 of Friedmann notes */
  /*   */
  /* #codejen */
  /*  Initialization */
  c_a = 0.0F;
  *T_ref = 21.311245F;
  fa = __anon_fcn(V_rel_B, rho, 0.0F);
  fb = __anon_fcn(V_rel_B, rho, 21.311245F);
  if (fa == 0.0F) {
    *T_ref = 0.0F;
  } else {
    if (!(fb == 0.0F)) {
      fc = fb;

      /*  Main loop, exit from middle of the loop */
      c = 0.0F;
      e = 0.0F;
      d = 0.0F;
      exitg1 = false;
      while ((!exitg1) && ((fb != 0.0F) && (c_a != *T_ref))) {
        /*  Insure that b is the best result so far, a is the previous */
        /*  value of b, and c is on the opposite side of the zero from b. */
        if ((fb > 0.0F) == (fc > 0.0F)) {
          c = c_a;
          fc = fa;
          d = *T_ref - c_a;
          e = d;
        }

        if (fabsf(fc) < fabsf(fb)) {
          c_a = *T_ref;
          *T_ref = c;
          c = c_a;
          fa = fb;
          fb = fc;
          fc = fa;
        }

        /*  Convergence test and possible exit */
        m = 0.5F * (c - *T_ref);

        /* toler = 2.0*tol*max(abs(b),1.0); */
        if ((fabsf(m) <= 0.001F) || (fb == 0.0F)) {
          exitg1 = true;
        } else {
          /*  Choose bisection or interpolation */
          if ((fabsf(e) < 0.001F) || (fabsf(fa) <= fabsf(fb))) {
            /*  Bisection */
            d = m;
            e = m;
          } else {
            /*  Interpolation */
            s = fb / fa;
            if (c_a == c) {
              /*  Linear interpolation */
              p = 2.0F * m * s;
              fa = 1.0F - s;
            } else {
              /*  Inverse quadratic interpolation */
              fa /= fc;
              r = fb / fc;
              p = s * (2.0F * m * fa * (fa - r) - (*T_ref - c_a) * (r - 1.0F));
              fa = (fa - 1.0F) * (r - 1.0F) * (s - 1.0F);
            }

            if (p > 0.0F) {
              fa = -fa;
            } else {
              p = -p;
            }

            /*  Is interpolated point acceptable */
            if ((2.0F * p < 3.0F * m * fa - fabsf(0.001F * fa)) && (p < fabsf
                 (0.5F * e * fa))) {
              e = d;
              d = p / fa;
            } else {
              d = m;
              e = m;
            }
          }

          /*  Interpolation */
          /*  Next point */
          c_a = *T_ref;
          fa = fb;
          if (fabsf(d) > 0.001F) {
            *T_ref += d;
          } else if (*T_ref > c) {
            *T_ref -= 0.001F;
          } else {
            *T_ref += 0.001F;
          }

          fb = __anon_fcn(V_rel_B, rho, *T_ref);
        }
      }
    }
  }

  /*  Main loop */
  /*  thrust coeffcient for a rotor */
  p = fabsf(alpha);
  c_a = sinf(p);
  fa = rho * 3.14159274F * 0.0103225596F * ((out + 0.0001F) * (out + 0.0001F));
  p = 0.5F * rho * 0.024F * alpha_tmp * 0.5F * cosf(alpha) + 4.0F * ((0.2601F *
    (0.25F * *T_ref / fa) * (c_a + 0.4837F) + 187.952194F * (a * a + b_a * b_a) *
    0.008F * 0.108383402F * (c_a + -0.1391F)) * cosf(p)) * fa;
  fa = p / (a_tmp + 1.0E-6F);
  *theta_ref = asinf(fa * V_rel_B[0] / 10.6556225F);
  *phi_ref = atanf(fa * V_rel_B[1] / -sqrtf(113.54229F - p * p));

  /*  */
  /* %%%%%%%%%%% using fsolve %%%%%%%%%%%%%% */
  /*  w=(1036.2+50)/1000*9.81;    */
  /*  fun=@(x) dynamic_eqns(x,rpm,V_rel_B,rho); */
  /*  x0 = [0,0,w]; */
  /*  x= fsolve(fun,x0); */
  /*   */
  /*  theta_ref=x(1); */
  /*  phi_ref=x(2); */
  /*  T=x(3); */
}

/*
 * File trailer for feedforward.c
 *
 * [EOF]
 */
