/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: thrust.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Sep-2020 23:59:27
 */

/* Include Files */
#include "thrust.h"
#include "trapz.h"
#include <arm_neon.h>
#include <math.h>
#include <string.h>

/* Function Declarations */
static float __anon_fcn(const float V_rel_B[3], float T, float nb, float rho,
  float M, const float th[36], const float c[36], const float cla[36], const
  float r[36], float rpm);

/* Function Definitions */

/*
 * Arguments    : const float V_rel_B[3]
 *                float T
 *                float nb
 *                float rho
 *                float M
 *                const float th[36]
 *                const float c[36]
 *                const float cla[36]
 *                const float r[36]
 *                float rpm
 * Return Type  : float
 */
static float __anon_fcn(const float V_rel_B[3], float T, float nb, float rho,
  float M, const float th[36], const float c[36], const float cla[36], const
  float r[36], float rpm)
{
  float varargout_1;
  float er1;
  float mu;
  float lamb_tot;
  float inv_pd_tmp;
  float inv_pd;
  float ct;
  float lamb;
  float lambnew;
  float32x4_t b_r;
  int ibmat;
  int ibcol;
  float psi_new[60];
  float lam[2160];
  float fcnOutput[60];
  float32x4_t r1;
  float psi2[2160];
  int itilerow;
  float ut[2160];
  float phi[2160];
  float b[2160];
  float sigma[2160];
  float b_b[2160];
  float y[2160];
  float b_fcnOutput[2160];
  float b_y[2160];
  float alp[2160];
  float fv[36];
  float32x4_t r2;
  float c_r[36];
  float b_nb[36];
  float32x4_t r3;
  float32x4_t r4;
  float32x4_t r5;
  float32x4_t r6;
  float32x4_t r7;
  float32x4_t r8;
  float32x4_t r9;

  /*  rotor characteristics */
  /*      Copyright (C) 12/12/2018  Regents of the University of Michigan */
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
  /* sig=nb*mean(c)/(pi*R);        % blade solidity */
  /*  */
  /*  relative incoming velocity */
  /*  2*pi/60 = 0.1047197 */
  /*  rad per second */
  er1 = rpm * 0.104719698F * 0.1016F;

  /*  tip velocity */
  mu = sqrtf(V_rel_B[0] * V_rel_B[0] + V_rel_B[1] * V_rel_B[1]) / (er1 + 0.0001F);

  /*  advance ratio */
  lamb_tot = V_rel_B[2] / (er1 + 0.0001F);

  /*  total external inflow ratio (effect lamb_c ans mu * tan alpha) */
  /*  the pitch angle of the quad-copter is already embded becuse the Vinf is */
  /*  in body frame! */
  /* alpha = atan(Vinf(3)/(norm(Vinf,2)+0.00001));            % copter angle of attack */
  /*  thrust coeffcient */
  inv_pd_tmp = (er1 + 0.0001F) * (er1 + 0.0001F);
  inv_pd = 1.0F / (rho * 3.14159274F * 0.0103225596F * inv_pd_tmp);

  /*  1/(2*dynamic pressure) */
  ct = T * inv_pd;

  /*  climb ratio, prependicular to free stream */
  /* lambc=lamb_tot/(cos(alpha)+0.0001); */
  /*  finding lamb0 */
  /*  lambda= mu*tan(alpha) + ct/(2*sqrt(mu^2+lambda(n)^2)) + lamc * cos(alpha) */
  er1 = 1.0F;

  /*  inflow for hover */
  lamb = sqrtf(0.5F * ct);

  /*  using exact solution for a more simplified case to behoove the initial guess */
  /* lamb = lah * ((0.25*(mu/lah)^4+1)^0.5-0.5*(mu/lah)^2)^0.5; */
  /*    lamb0 = mu * tan (alpha) + lambc * cos(alpha) + 0.5 *  ct * (mu^2 + lamb0 ^2) ^ (-0.5) ; */
  /*    it is noted that lamb_tot = mu * tan (alpha) + lambc * cos(alpha) */
  /*  induced inflow ratio from momemntum theory */
  while (er1 > 0.005F) {
    /*     f = lamb0 - mu * tan (alpha) - lambc * cos(alpha) - 0.5 *  ct * (mu^2 + lamb0 ^2) ^ (-0.5) ; */
    er1 = 0.5F * ct;
    lambnew = mu * mu + lamb * lamb;
    lambnew = lamb - ((lamb - lamb_tot) - er1 * powf(lambnew, -0.5F)) / (er1 *
      powf(lambnew, -1.5F) * lamb + 1.0F);
    er1 = fabsf((lambnew - lamb) / lamb);
    lamb = lambnew;
  }

  /* lambda_one=lamb0; */
  /* dct=2*lamb0^2-ct; */
  /*  page 158 - 160 */
  /*  muz is the advance ratio prependicular to the rotor (lambda_c) */
  /*  mux is the advance ratio parallel to the rotor */
  /*  muz = lamb_tot; */
  /*  wake skew angle */
  /*  different linear inflow models */
  /*  Drees: */
  /*  kx = 4/3 * (1 - cos(x) - 1.8 * mu^2) / sin(x) ; */
  /*   */
  /*  if x==0 */
  /*      kx=0; */
  /*  end */
  /*   */
  /*  ky = -2*mu; */
  /*  Pitt and Peters: */
  /*  15*pi/23 = 2.048864 */
  lambnew = 2.04886389F * tanf(atanf(mu / lamb) / 2.0F);

  /*  fixing psi given a Vy velcoity, psi is defined when x is alinged with free stream */
  er1 = atanf(-V_rel_B[1] / (V_rel_B[0] + 0.0001F));

  /*  constructing inflow ratio based on a linear model */
  /*  lambda is a matrix, rows r, and columns azimuth, (nr * npsi) */
  b_r = vdupq_n_f32(0.0F);
  for (ibmat = 0; ibmat < 60; ibmat++) {
    lamb_tot = 0.106494673F * (float)ibmat - er1;
    psi_new[ibmat] = lamb_tot;
    fcnOutput[ibmat] = cosf(lamb_tot);
    for (ibcol = 0; ibcol <= 32; ibcol += 4) {
      itilerow = ibcol + 36 * ibmat;
      vst1q_f32(&lam[itilerow], vaddq_f32(vaddq_f32(b_r, vmulq_f32(vmulq_f32
        (vdupq_n_f32(lambnew), vld1q_f32(&r[ibcol])), vdupq_n_f32
        (fcnOutput[ibmat]))), vdupq_n_f32(1.0F)));
      vst1q_f32(&psi2[itilerow], b_r);
    }
  }

  for (ibcol = 0; ibcol <= 2156; ibcol += 4) {
    b_r = vld1q_f32(&lam[ibcol]);
    r1 = vld1q_f32(&psi2[ibcol]);
    vst1q_f32(&lam[ibcol], vmulq_f32(vdupq_n_f32(lamb), vaddq_f32(b_r, r1)));
  }

  /*  intergration to find roll and pitch for a rotor */
  /*  nr*npsi */
  for (ibcol = 0; ibcol < 60; ibcol++) {
    ibmat = ibcol * 36;
    for (itilerow = 0; itilerow < 36; itilerow++) {
      psi2[ibmat + itilerow] = psi_new[ibcol];
    }
  }

  /*  nr*npsi */
  /*  the incoming velocity seen by the blade */
  /*  for a quad-copter (no flpapping) */
  for (ibmat = 0; ibmat < 2160; ibmat++) {
    psi2[ibmat] = sinf(psi2[ibmat]);
  }

  for (ibmat = 0; ibmat < 60; ibmat++) {
    ibcol = ibmat * 36;
    memcpy(&ut[ibcol], &r[0], 36U * sizeof(float));
  }

  /* u = (ut.^2 +up.^2) .^0.5 ; */
  /* usq = (ut.^2 +up.^2);  %u^2 */
  /*  matrix form of the variables */
  for (ibmat = 0; ibmat < 2160; ibmat++) {
    lamb_tot = ut[ibmat] + mu * psi2[ibmat];
    ut[ibmat] = lamb_tot;
    phi[ibmat] = atanf(lam[ibmat] / lamb_tot);
  }

  /*  nr*npsi */
  /*  nr*npsi */
  /*  nr*npsi */
  for (ibmat = 0; ibmat < 60; ibmat++) {
    ibcol = ibmat * 36;
    memcpy(&b[ibcol], &th[0], 36U * sizeof(float));
  }

  /*  based on Beard-McLain book page 47 to model stall condition */
  /*  180/pi = 57.2957795 */
  /*  lift generated by every annulus */
  /*  intergral int_0^{2pi} cl * 1/2 * rho * U^2 * c2 / (2pi) */
  /*  l = nb * trapz(psi_new, cl * 0.5 .* rho .* (vt*u).^2 .*c2 , 2) / (2*pi); */
  /*  (1/ 2pi) = 0.1591549 */
  for (ibmat = 0; ibmat < 2160; ibmat++) {
    lamb_tot = b[ibmat] - phi[ibmat];
    b[ibmat] = lamb_tot;
    er1 = expf(M * ((lamb_tot - 0.0349065848F) + 0.35953784F));
    lambnew = expf(-M * ((lamb_tot - 0.0349065848F) - 0.35953784F)) + 1.0F;
    lambnew = (lambnew + er1) / (lambnew * (er1 + 1.0F));
    sigma[ibmat] = lambnew;
    er1 = lamb_tot - 0.0349065848F;
    if (lamb_tot - 0.0349065848F < 0.0F) {
      er1 = -1.0F;
    } else {
      if (lamb_tot - 0.0349065848F > 0.0F) {
        er1 = 1.0F;
      }
    }

    b_fcnOutput[ibmat] = er1;
    psi2[ibmat] = sinf(lamb_tot - 0.0349065848F);
    alp[ibmat] = cosf(lamb_tot - 0.0349065848F);
    phi[ibmat] = cosf(phi[ibmat]);
  }

  /*  note that vt wil be cancel out in ct calculations */
  /*  the error parameter defined between momentum and blade element theories */
  /*  Golden search: */
  /* er_ct=abs(ct_two-ct)/ct; */
  /*  use this if used fzero */
  for (ibmat = 0; ibmat < 60; ibmat++) {
    ibcol = ibmat * 36;
    memcpy(&b_b[ibcol], &cla[0], 36U * sizeof(float));
  }

  for (ibmat = 0; ibmat <= 2156; ibmat += 4) {
    b_r = vld1q_f32(&psi2[ibmat]);
    vst1q_f32(&y[ibmat], vmulq_f32(b_r, b_r));
    b_r = vld1q_f32(&ut[ibmat]);
    vst1q_f32(&b_y[ibmat], vmulq_f32(b_r, b_r));
    b_r = vld1q_f32(&lam[ibmat]);
    vst1q_f32(&ut[ibmat], vmulq_f32(b_r, b_r));
  }

  for (ibmat = 0; ibmat < 60; ibmat++) {
    ibcol = ibmat * 36;
    memcpy(&psi2[ibcol], &c[0], 36U * sizeof(float));
  }

  for (ibcol = 0; ibcol <= 2156; ibcol += 4) {
    b_r = vld1q_f32(&sigma[ibcol]);
    r1 = vld1q_f32(&b_b[ibcol]);
    r2 = vld1q_f32(&b[ibcol]);
    r3 = vld1q_f32(&b_fcnOutput[ibcol]);
    r4 = vld1q_f32(&y[ibcol]);
    r5 = vld1q_f32(&alp[ibcol]);
    r6 = vld1q_f32(&b_y[ibcol]);
    r7 = vld1q_f32(&ut[ibcol]);
    r8 = vld1q_f32(&psi2[ibcol]);
    r9 = vld1q_f32(&phi[ibcol]);
    vst1q_f32(&sigma[ibcol], vmulq_f32(vmulq_f32(vmulq_f32(vmulq_f32(vmulq_f32
      (vmulq_f32(vaddq_f32(vmulq_f32(vsubq_f32(vdupq_n_f32(1.0F), b_r),
      vmulq_f32(r1, r2)), vmulq_f32(b_r, vmulq_f32(vmulq_f32(vmulq_f32
      (vdupq_n_f32(2.0F), r3), r4), r5))), vdupq_n_f32(0.5F)), vdupq_n_f32(rho)),
      vdupq_n_f32(inv_pd_tmp)), vaddq_f32(r6, r7)), r8), r9));
  }

  trapz(psi_new, sigma, fv);
  for (ibcol = 0; ibcol <= 32; ibcol += 4) {
    vst1q_f32(&c_r[ibcol], vmulq_f32(vld1q_f32(&r[ibcol]), vdupq_n_f32(0.1016F)));
    b_r = vld1q_f32(&fv[ibcol]);
    vst1q_f32(&b_nb[ibcol], vmulq_f32(vmulq_f32(vdupq_n_f32(nb), b_r),
               vdupq_n_f32(0.159154907F)));
  }

  varargout_1 = (b_trapz(c_r, b_nb) * inv_pd - ct) / ct;

  /*  blade profile drag */
  /*  Cd0=0.0081-0.0216*aoa+0.4*aoa^2; from helicopter literature, note aoa is the angle of attack - page 14, chapter 2 of Friedmann notes */
  /*  Cd0=0.008; */
  /*  f is the equivalent flat plate area that copter frame occupies in space */
  /*  -- note that a quarter of the area should be used since this is model for */
  /*  for a helicopter, f/A is between 0.004 to 0.025 */
  /*  f=0.005*A * 0.25; */
  /* cmyaw=0; */
  /*  cmyaw=sig * Cd0 * 0.125 * (1 + 4.6* mu^2) + ... */
  /*       1.15 * 0.5 * ct_two^2 / sqrt(mu^2+lamb^2) + ... */
  /*       f * mu^3 /(2*pi*R^2) +... */
  /*       ct_two*lambc ; */
  /*  yaw=cmyaw*(rho*pi*R^3*vt^2); */
  /*  nu=1.534e-5; */
  /*  Re=u.*c2*vt*65189.048; */
  /*  x point to the nose of the vehicle, y to the right wing (pilot's right), */
  /*  z pointing downward - right wing dowm, nose up, right wing back are */
  /*  positive roll, pitch, yaw moments. psi = 0, starts farther from the */
  /*  inflow, x<0 */
  /*  x=-r2.*cos(psi2)*R; */
  /*  y=r2.*sin(psi2)*R; */
  /*  1/(2*pi) = 0.15915494 */
  /*  roll = trapz(r*R,nb* 0.15915494*trapz(psi_new,-y.*term,2)); */
  /*  pitch = trapz(r*R,nb* 0.15915494*trapz(psi_new,x.*term,2)); */
  return varargout_1;
}

/*
 * #codejen
 * Arguments    : float rpm_low
 *                float rpm_high
 *                float T
 *                float rho
 *                const float V_rel_B[3]
 * Return Type  : float
 */
float thrust(float rpm_low, float rpm_high, float T, float rho, const float
             V_rel_B[3])
{
  float rpm;
  float a;
  int i;
  float fa;
  static const float fv[36] = { 0.745006561F, 0.730006576F, 0.708706558F,
    0.683106542F, 0.658606589F, 0.635706544F, 0.605806589F, 0.568206549F,
    0.534606576F, 0.504806578F, 0.478006601F, 0.454006582F, 0.432306588F,
    0.412606597F, 0.394706607F, 0.378406584F, 0.363406599F, 0.34970659F,
    0.337006599F, 0.325206608F, 0.314406604F, 0.30420661F, 0.2948066F,
    0.2860066F, 0.27780658F, 0.270106584F, 0.262806594F, 0.256006598F,
    0.24960658F, 0.243506581F, 0.237706587F, 0.232306585F, 0.227106586F,
    0.222206578F, 0.217906579F, 0.213806584F };

  static const float fv1[36] = { 0.0176F, 0.0184F, 0.019F, 0.0196F, 0.0202F,
    0.0207F, 0.0213F, 0.022F, 0.0224F, 0.0227F, 0.0227F, 0.0225F, 0.0223F,
    0.022F, 0.0216F, 0.0213F, 0.0208F, 0.0204F, 0.0199F, 0.0193F, 0.0187F,
    0.0181F, 0.0175F, 0.0168F, 0.0161F, 0.0153F, 0.0145F, 0.0137F, 0.0129F,
    0.012F, 0.0112F, 0.0103F, 0.0094F, 0.0084F, 0.0071F, 0.004F };

  float fv2[36];
  static const float fv3[36] = { 0.1991F, 0.2116F, 0.224F, 0.2364F, 0.2488F,
    0.2613F, 0.2788F, 0.3033F, 0.328F, 0.3526F, 0.3773F, 0.4019F, 0.4266F,
    0.4513F, 0.4759F, 0.5006F, 0.5253F, 0.5499F, 0.5746F, 0.5993F, 0.6239F,
    0.6486F, 0.6733F, 0.6979F, 0.7226F, 0.7472F, 0.7719F, 0.7966F, 0.8213F,
    0.8459F, 0.8706F, 0.8952F, 0.9199F, 0.9446F, 0.9676F, 0.9901F };

  float fb;
  float fc;
  float c;
  float e;
  float d;
  bool exitg1;
  float m;
  float s;
  float q;
  float r;

  /*  propeller radius  [m] */
  /*  number of blade */
  /*  rotor disk area */
  /*  stall transition rate */
  /*  stall cut-off aoa */
  /*  absolute values of angle of attack where lift is zero */
  /*  blade characteristics */
  /* th=[MR8x45(:,8)]*pi/180+aLeq0;% aLeq0 must be added here! */
  /* r=MR8x45(:,1)/4;                       % normolized radial locations, R=4in */
  /*  number of data point in azimuth */
  /*  2-D lift curve slope */
  /*  azimuth angle */
  /* #codejen */
  /*  Initialization */
  a = rpm_low;
  rpm = rpm_high;
  for (i = 0; i < 36; i++) {
    fv2[i] = 5.6548667F;
  }

  fa = __anon_fcn(V_rel_B, T, 2.0F, rho, 50.0F, fv, fv1, fv2, fv3, rpm_low);
  for (i = 0; i < 36; i++) {
    fv2[i] = 5.6548667F;
  }

  fb = __anon_fcn(V_rel_B, T, 2.0F, rho, 50.0F, fv, fv1, fv2, fv3, rpm_high);
  if (fa == 0.0F) {
    rpm = rpm_low;
  } else {
    if (fb != 0.0F) {
      fc = fb;

      /*  Main loop, exit from middle of the loop */
      c = 0.0F;
      e = 0.0F;
      d = 0.0F;
      exitg1 = false;
      while ((!exitg1) && ((fb != 0.0F) && (a != rpm))) {
        /*  Insure that b is the best result so far, a is the previous */
        /*  value of b, and c is on the opposite side of the zero from b. */
        if ((fb > 0.0F) == (fc > 0.0F)) {
          c = a;
          fc = fa;
          d = rpm - a;
          e = d;
        }

        if (fabsf(fc) < fabsf(fb)) {
          a = rpm;
          rpm = c;
          c = a;
          fa = fb;
          fb = fc;
          fc = fa;
        }

        /*  Convergence test and possible exit */
        m = 0.5F * (c - rpm);

        /* toler = 2.0*tol*max(abs(b),1.0); */
        if ((fabsf(m) <= 20.0F) || (fb == 0.0F)) {
          exitg1 = true;
        } else {
          /*  Choose bisection or interpolation */
          if ((fabsf(e) < 20.0F) || (fabsf(fa) <= fabsf(fb))) {
            /*  Bisection */
            d = m;
            e = m;
          } else {
            /*  Interpolation */
            s = fb / fa;
            if (a == c) {
              /*  Linear interpolation */
              fa = 2.0F * m * s;
              q = 1.0F - s;
            } else {
              /*  Inverse quadratic interpolation */
              q = fa / fc;
              r = fb / fc;
              fa = s * (2.0F * m * q * (q - r) - (rpm - a) * (r - 1.0F));
              q = (q - 1.0F) * (r - 1.0F) * (s - 1.0F);
            }

            if (fa > 0.0F) {
              q = -q;
            } else {
              fa = -fa;
            }

            /*  Is interpolated point acceptable */
            if ((2.0F * fa < 3.0F * m * q - fabsf(20.0F * q)) && (fa < fabsf
                 (0.5F * e * q))) {
              e = d;
              d = fa / q;
            } else {
              d = m;
              e = m;
            }
          }

          /*  Interpolation */
          /*  Next point */
          a = rpm;
          fa = fb;
          if (fabsf(d) > 20.0F) {
            rpm += d;
          } else if (rpm > c) {
            rpm -= 20.0F;
          } else {
            rpm += 20.0F;
          }

          for (i = 0; i < 36; i++) {
            fv2[i] = 5.6548667F;
          }

          fb = __anon_fcn(V_rel_B, T, 2.0F, rho, 50.0F, fv, fv1, fv2, fv3, rpm);
        }
      }
    }
  }

  /*  Main loop */
  return rpm;
}

/*
 * File trailer for thrust.c
 *
 * [EOF]
 */
