/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: thrust.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 05-Jan-2021 07:14:57
 */

/* Include Files */
#include "thrust.h"
#include "mytrapz.h"
#include <arm_neon.h>
#include <math.h>

/* Function Declarations */
static float __anon_fcn(const float V_rel_B[3], float T, float rho, float M,
  const float c2[234], const float psi2[234], const float rprime_cos_psinew[234],
  const float psi_new[18], float rpm);

/* Function Definitions */

/*
 * Arguments    : const float V_rel_B[3]
 *                float T
 *                float rho
 *                float M
 *                const float c2[234]
 *                const float psi2[234]
 *                const float rprime_cos_psinew[234]
 *                const float psi_new[18]
 *                float rpm
 * Return Type  : float
 */
static float __anon_fcn(const float V_rel_B[3], float T, float rho, float M,
  const float c2[234], const float psi2[234], const float rprime_cos_psinew[234],
  const float psi_new[18], float rpm)
{
  float er1;
  float mu;
  float lamb_tot;
  float inv_pd_tmp;
  float inv_pd;
  float ct;
  float lamb;
  float lambnew;
  int k;
  float sigma[234];
  float fv[13];
  float32x4_t r;
  static const float fv1[234] = { 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F,
    0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F,
    0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F,
    0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F,
    0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F,
    0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F,
    0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F,
    0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F,
    0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F,
    0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F,
    0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F,
    0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F,
    0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F,
    0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F,
    0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F,
    0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F,
    0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F,
    0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F,
    0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F,
    0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F,
    0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F,
    0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F,
    0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F,
    0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F,
    0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F,
    0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F,
    0.8706F, 0.9446F, 0.9901F, 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F,
    0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F,
    0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F, 0.5006F, 0.5746F, 0.6486F,
    0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F };

  float32x4_t r1;
  float f;
  float32x4_t r2;
  float f1;
  static const float fv2[234] = { 0.745006561F, 0.683106542F, 0.605806589F,
    0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F, 0.30420661F,
    0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F, 0.213806584F,
    0.745006561F, 0.683106542F, 0.605806589F, 0.504806578F, 0.432306588F,
    0.378406584F, 0.337006599F, 0.30420661F, 0.27780658F, 0.256006598F,
    0.237706587F, 0.222206578F, 0.213806584F, 0.745006561F, 0.683106542F,
    0.605806589F, 0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F,
    0.30420661F, 0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F,
    0.213806584F, 0.745006561F, 0.683106542F, 0.605806589F, 0.504806578F,
    0.432306588F, 0.378406584F, 0.337006599F, 0.30420661F, 0.27780658F,
    0.256006598F, 0.237706587F, 0.222206578F, 0.213806584F, 0.745006561F,
    0.683106542F, 0.605806589F, 0.504806578F, 0.432306588F, 0.378406584F,
    0.337006599F, 0.30420661F, 0.27780658F, 0.256006598F, 0.237706587F,
    0.222206578F, 0.213806584F, 0.745006561F, 0.683106542F, 0.605806589F,
    0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F, 0.30420661F,
    0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F, 0.213806584F,
    0.745006561F, 0.683106542F, 0.605806589F, 0.504806578F, 0.432306588F,
    0.378406584F, 0.337006599F, 0.30420661F, 0.27780658F, 0.256006598F,
    0.237706587F, 0.222206578F, 0.213806584F, 0.745006561F, 0.683106542F,
    0.605806589F, 0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F,
    0.30420661F, 0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F,
    0.213806584F, 0.745006561F, 0.683106542F, 0.605806589F, 0.504806578F,
    0.432306588F, 0.378406584F, 0.337006599F, 0.30420661F, 0.27780658F,
    0.256006598F, 0.237706587F, 0.222206578F, 0.213806584F, 0.745006561F,
    0.683106542F, 0.605806589F, 0.504806578F, 0.432306588F, 0.378406584F,
    0.337006599F, 0.30420661F, 0.27780658F, 0.256006598F, 0.237706587F,
    0.222206578F, 0.213806584F, 0.745006561F, 0.683106542F, 0.605806589F,
    0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F, 0.30420661F,
    0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F, 0.213806584F,
    0.745006561F, 0.683106542F, 0.605806589F, 0.504806578F, 0.432306588F,
    0.378406584F, 0.337006599F, 0.30420661F, 0.27780658F, 0.256006598F,
    0.237706587F, 0.222206578F, 0.213806584F, 0.745006561F, 0.683106542F,
    0.605806589F, 0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F,
    0.30420661F, 0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F,
    0.213806584F, 0.745006561F, 0.683106542F, 0.605806589F, 0.504806578F,
    0.432306588F, 0.378406584F, 0.337006599F, 0.30420661F, 0.27780658F,
    0.256006598F, 0.237706587F, 0.222206578F, 0.213806584F, 0.745006561F,
    0.683106542F, 0.605806589F, 0.504806578F, 0.432306588F, 0.378406584F,
    0.337006599F, 0.30420661F, 0.27780658F, 0.256006598F, 0.237706587F,
    0.222206578F, 0.213806584F, 0.745006561F, 0.683106542F, 0.605806589F,
    0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F, 0.30420661F,
    0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F, 0.213806584F,
    0.745006561F, 0.683106542F, 0.605806589F, 0.504806578F, 0.432306588F,
    0.378406584F, 0.337006599F, 0.30420661F, 0.27780658F, 0.256006598F,
    0.237706587F, 0.222206578F, 0.213806584F, 0.745006561F, 0.683106542F,
    0.605806589F, 0.504806578F, 0.432306588F, 0.378406584F, 0.337006599F,
    0.30420661F, 0.27780658F, 0.256006598F, 0.237706587F, 0.222206578F,
    0.213806584F };

  float fv3[13];
  float f2;
  float f3;
  float x;
  static const float fv4[13] = { 0.0202285592F, 0.0240182392F, 0.0283260811F,
    0.035824161F, 0.0433425605F, 0.0508609563F, 0.0583793558F, 0.0658977553F,
    0.0734161586F, 0.0809345543F, 0.0884529576F, 0.0959713608F, 0.100594163F };

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
  /*  mux = mu ; */
  /*  muz = lamb_tot; */
  /*  wake skew angle */
  /*  Pitt and Peters: */
  /*  15*pi/23 = 2.048864 */
  er1 = 2.04886389F * tanf(atanf(mu / lamb) * 0.5F);

  /*  ky = 0; */
  /*  fixing psi given a Vy velcoity, psi is defined when x is alinged with free stream */
  /*  psi_new=psi-atan(-Vy/(Vx+0.0001)); */
  /*  constructing inflow ratio based on a linear model */
  /*  lambda is a matrix, rows r, and columns azimuth, (nr * npsi) */
  /*  intergration to find roll and pitch for a rotor */
  /*  r2=repmat(r',1,npsi);        % nr*npsi */
  /*  psi2=repmat(psi_new,nr,1);       % nr*npsi */
  /*  the incoming velocity seen by the blade */
  /*  for a quad-copter (no flpapping) */
  /* u = (ut.^2 +up.^2) .^0.5 ; */
  /* usq = (ut.^2 +up.^2);  %u^2 */
  /*  matrix form of the variables */
  /* we can take this out of try11 */
  /*  th2=repmat(th',1,npsi);      % nr*npsi */
  /*  cla2=repmat(cla',1,npsi);     % nr*npsi */
  /*  c2=repmat(c',1,npsi);        % nr*npsi */
  /*  based on Beard-McLain book page 47 to model stall condition */
  /*  180/pi = 57.2957795 */
  /*  lift generated by every annulus */
  /*  intergral int_0^{2pi} cl * 1/2 * rho * U^2 * c2 / (2pi) */
  /*  l = nb * trapz(psi_new, cl * 0.5 .* rho .* (vt*u).^2 .*c2 , 2) / (2*pi); */
  /*  (1/ 2pi) = 0.1591549 */
  /*  note that vt wil be cancel out in ct calculations */
  /*  the error parameter defined between momentum and blade element theories */
  /*  Golden search: */
  /* er_ct=abs(ct_two-ct)/ct; */
  /*  use this if used fzero */
  for (k = 0; k < 234; k++) {
    lambnew = lamb * (er1 * rprime_cos_psinew[k] + 1.0F);
    lamb_tot = fv1[k] + mu * sinf(psi2[k]);
    f = atanf(lambnew / lamb_tot);
    f1 = fv2[k] - f;
    f2 = expf(M * ((f1 - 0.0349065848F) + 0.35953784F));
    f3 = expf(-M * ((f1 - 0.0349065848F) - 0.35953784F)) + 1.0F;
    f3 = (f3 + f2) / (f3 * (f2 + 1.0F));
    x = f1 - 0.0349065848F;
    if (f1 - 0.0349065848F < 0.0F) {
      x = -1.0F;
    } else {
      if (f1 - 0.0349065848F > 0.0F) {
        x = 1.0F;
      }
    }

    f2 = sinf(f1 - 0.0349065848F);
    f = cosf(f);
    f3 = ((1.0F - f3) * (5.6548667F * f1) + f3 * (2.0F * x * (f2 * f2) * cosf(f1
            - 0.0349065848F))) * 0.5F * rho * inv_pd_tmp * (lamb_tot * lamb_tot
      + lambnew * lambnew) * c2[k] * f;
    sigma[k] = f3;
  }

  mytrapz(psi_new, sigma, fv);
  r = vld1q_f32(&fv[0]);
  r1 = vdupq_n_f32(2.0F);
  r2 = vdupq_n_f32(0.159154907F);
  vst1q_f32(&fv3[0], vmulq_f32(vmulq_f32(r1, r), r2));
  r = vld1q_f32(&fv[4]);
  vst1q_f32(&fv3[4], vmulq_f32(vmulq_f32(r1, r), r2));
  r = vld1q_f32(&fv[8]);
  vst1q_f32(&fv3[8], vmulq_f32(vmulq_f32(r1, r), r2));
  fv3[12] = 2.0F * fv[12] * 0.159154907F;
  return (b_mytrapz(fv4, fv3) * inv_pd - ct) / ct;
}

/*
 * #codejen
 * Arguments    : float rpm_low
 *                float rpm_high
 *                float T
 *                float rho
 *                const float V_rel_B[3]
 *                float *rpm
 *                float *fakerpm
 * Return Type  : void
 */
void thrust(float rpm_low, float rpm_high, float T, float rho, const float
            V_rel_B[3], float *rpm)
{
  float fa;
  int jcol;
  float p;
  float psi_new[18];
  int ibmat;
  float a;
  float32x4_t r;
  float psi2[234];
  float32x4_t r1;
  static const float fv[234] = { 0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F,
    0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F,
    0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F,
    0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F,
    0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F,
    0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F,
    0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F,
    0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F,
    0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F,
    0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F,
    0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F,
    0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F,
    0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F,
    0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F,
    0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F,
    0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F,
    0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F,
    0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F,
    0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F,
    0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F,
    0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F,
    0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F,
    0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F,
    0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F,
    0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F,
    0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F,
    0.0112F, 0.0084F, 0.004F, 0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F,
    0.0213F, 0.0199F, 0.0181F, 0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F,
    0.0176F, 0.0196F, 0.0213F, 0.0227F, 0.0223F, 0.0213F, 0.0199F, 0.0181F,
    0.0161F, 0.0137F, 0.0112F, 0.0084F, 0.004F };

  float subfcn2_tunableEnvironment_f5[234];
  static const float b_a[13] = { 0.1991F, 0.2364F, 0.2788F, 0.3526F, 0.4266F,
    0.5006F, 0.5746F, 0.6486F, 0.7226F, 0.7966F, 0.8706F, 0.9446F, 0.9901F };

  float fb;
  float fc;
  float c;
  float e;
  float d;
  bool exitg1;
  float m;
  float s;
  float b_r;

  /*  did this with two outputs so that Codgen makes a pointer type output */
  /*  propeller radius  [m] */
  /*  number of blade */
  /* A=pi*R^2;                            % rotor disk area */
  /*  stall transition rate */
  /*  stall cut-off aoa */
  /*  absolute values of angle of attack where lift is zero */
  /*  blade characteristics */
  /* th=[MR8x45(:,8)]*pi/180+aLeq0;% aLeq0 must be added here! */
  /* r=MR8x45(:,1)/4;                       % normolized radial locations, R=4in */
  /*  number of data point in azimuth */
  /*  2-D lift curve slope */
  /*  azimuth angle */
  /*  */
  /*  nr*npsi */
  /*  nr*npsi */
  /*  nr*npsi */
  /*  nr*npsi */
  fa = atanf(-V_rel_B[1] / (V_rel_B[0] + 0.0001F));

  /*  nr*npsi */
  for (jcol = 0; jcol < 18; jcol++) {
    p = 0.369599134F * (float)jcol - fa;
    psi_new[jcol] = p;
    ibmat = jcol * 13;
    a = cosf(p);
    r = vdupq_n_f32(p);
    vst1q_f32(&psi2[ibmat], r);
    r1 = vdupq_n_f32(a);
    vst1q_f32(&subfcn2_tunableEnvironment_f5[13 * jcol], vmulq_f32(vld1q_f32
               (&b_a[0]), r1));
    vst1q_f32(&psi2[ibmat + 4], r);
    vst1q_f32(&subfcn2_tunableEnvironment_f5[13 * jcol + 4], vmulq_f32(vld1q_f32
               (&b_a[4]), r1));
    vst1q_f32(&psi2[ibmat + 8], r);
    vst1q_f32(&subfcn2_tunableEnvironment_f5[13 * jcol + 8], vmulq_f32(vld1q_f32
               (&b_a[8]), r1));
    psi2[ibmat + 12] = p;
    subfcn2_tunableEnvironment_f5[13 * jcol + 12] = b_a[12] * a;
  }

  /*  old way of doing it */
  /* subfcn2 = @(rpm) try11(rpm,V_rel_B,T,R,nb,A,rho,nr,npsi,M,alp0,aLeq0,th',c',cla,r',psi); */
  /*  */
  /* #codejen */
  /*  Initialization */
  a = rpm_low;
  *rpm = rpm_high;
  fa = __anon_fcn(V_rel_B, T, rho, 50.0F, fv, psi2,
                  subfcn2_tunableEnvironment_f5, psi_new, rpm_low);
  fb = __anon_fcn(V_rel_B, T, rho, 50.0F, fv, psi2,
                  subfcn2_tunableEnvironment_f5, psi_new, rpm_high);
  if (fa == 0.0F) {
    *rpm = rpm_low;
  } else {
    if (fb != 0.0F) {
      fc = fb;

      /*  Main loop, exit from middle of the loop */
      c = 0.0F;
      e = 0.0F;
      d = 0.0F;
      exitg1 = false;
      while ((!exitg1) && ((fb != 0.0F) && (a != *rpm))) {
        /*  Insure that b is the best result so far, a is the previous */
        /*  value of b, and c is on the opposite side of the zero from b. */
        if ((fb > 0.0F) == (fc > 0.0F)) {
          c = a;
          fc = fa;
          d = *rpm - a;
          e = d;
        }

        if (fabsf(fc) < fabsf(fb)) {
          a = *rpm;
          *rpm = c;
          c = a;
          fa = fb;
          fb = fc;
          fc = fa;
        }

        /*  Convergence test and possible exit */
        m = 0.5F * (c - *rpm);

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
              p = 2.0F * m * s;
              fa = 1.0F - s;
            } else {
              /*  Inverse quadratic interpolation */
              fa /= fc;
              b_r = fb / fc;
              p = s * (2.0F * m * fa * (fa - b_r) - (*rpm - a) * (b_r - 1.0F));
              fa = (fa - 1.0F) * (b_r - 1.0F) * (s - 1.0F);
            }

            if (p > 0.0F) {
              fa = -fa;
            } else {
              p = -p;
            }

            /*  Is interpolated point acceptable */
            if ((2.0F * p < 3.0F * m * fa - fabsf(20.0F * fa)) && (p < fabsf
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
          a = *rpm;
          fa = fb;
          if (fabsf(d) > 20.0F) {
            *rpm += d;
          } else if (*rpm > c) {
            *rpm -= 20.0F;
          } else {
            *rpm += 20.0F;
          }

          fb = __anon_fcn(V_rel_B, T, rho, 50.0F, fv, psi2,
                          subfcn2_tunableEnvironment_f5, psi_new, *rpm);
        }
      }
    }
  }

  /*  Main loop */
}

/*
 * File trailer for thrust.c
 *
 * [EOF]
 */
