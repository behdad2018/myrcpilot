/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: circl_traj.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 14-Dec-2020 19:37:44
 */

/* Include Files */
#include "circl_traj.h"
#include <math.h>

/* Function Definitions */

/*
 * #codejen
 * Arguments    : float x0
 *                float b_y0
 *                float z0
 *                float counter
 *                float R
 *                float Vcruise
 *                float nrev
 *                float t_takeoff
 *                float t_delay
 *                float z_target
 *                float f
 *                float *x
 *                float *y
 *                float *z
 * Return Type  : void
 */
void circl_traj(float x0, float b_y0, float z0, float counter, float R, float
                Vcruise, float nrev, float t_takeoff, float t_delay, float
                z_target, float f, float *x, float *y, float *z)
{
  float t_offset;
  float omega_cruise;
  float t_acc;
  float t_rev;
  float subcounter;

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
  /*  this code is written to be used in Mair. Assuming the take off from the */
  /*  center of MAir, a circle will be tracked. */
  /*  inputs to function */
  /*  x0,y0,z0      initial position of the drone sent in by motion capture system */
  /*  counter       is an index that is used for tracking */
  /*  R             radius of the circle */
  /*  Vcruise       tangential cruise speed on the circle trajectory */
  /*  nrev          # of revolutions traversed after reaching to Vcruise in one cycle */
  /*                total number of revolutions will be nrev+2 */
  /*  t_takeoff=2;                    % time needed to take off to z_target */
  /*  t_delay=2;                      % time over which the drones need to stand still */
  /*  z_target=-1;                    % target altitude */
  /*  f=200;                          % frequency at which this code will be executed */
  /*  outputs of the function */
  /*  x,y,z         target locations of the drone  */
  /*  definitions */
  t_offset = R / (Vcruise / 3.0F);

  /*  time needed to move from the center to the perimeter of the circle */
  omega_cruise = Vcruise / R;

  /*  the cruise angular velocity */
  t_acc = 12.5664F / omega_cruise;

  /*  acc and dec time that takes exactly one full revolution */
  t_rev = 6.2832F / omega_cruise;

  /*  time to do one rev at cruise speed/omega */
  /*  trajectory defnitions */

  /*  take off  */
  subcounter = f * t_takeoff;
  if (counter <= subcounter) {
    *x = x0;
    *y = b_y0;
    *z = z_target / subcounter * counter + z0;

    /*  wait after take off */
  } else {
    if ((subcounter >= counter) || (counter > f * (t_takeoff + t_delay))) {
      subcounter = f * (t_takeoff + t_delay);
      if ((subcounter < counter) && (counter <= f * ((t_takeoff + t_delay) +
            t_offset))) {
        *x = x0 + R / t_offset * (counter - subcounter) / f;
        *y = b_y0;
        *z = z_target + z0;

        /*  wait before the circle starts */
      } else {
        if ((f * ((t_takeoff + t_delay) + t_offset) >= counter) || (counter > f *
             ((t_takeoff + 2.0F * t_delay) + t_offset))) {
          subcounter = f * ((t_takeoff + 2.0F * t_delay) + t_offset);
          if ((subcounter < counter) && (counter <= f * (((t_takeoff + 2.0F *
                  t_delay) + t_offset) + t_acc))) {
            t_offset = (counter - subcounter) / f;
            subcounter = 0.5F * (omega_cruise * omega_cruise) / 12.5664F *
              (t_offset * t_offset);
            *x = x0 + R * cosf(subcounter);
            *y = b_y0 + R * sinf(subcounter);
            *z = z0 + z_target;

            /*  cruising at Vcruise that we now have reached */
          } else {
            subcounter = f * (((t_takeoff + 2.0F * t_delay) + t_offset) + t_acc);
            if ((subcounter < counter) && (counter <= f * ((((t_takeoff + 2.0F *
                     t_delay) + t_offset) + t_acc) + nrev * t_rev))) {
              subcounter = omega_cruise * (counter - subcounter) / f;
              *x = x0 + R * cosf(subcounter);
              *y = b_y0 + R * sinf(subcounter);
              *z = z0 + z_target;

              /*  deceleration phase on the circle done in one cycle */
            } else {
              subcounter = f * ((((t_takeoff + 2.0F * t_delay) + t_offset) +
                                 t_acc) + nrev * t_rev);
              if ((subcounter < counter) && (counter <= f * ((((t_takeoff + 2.0F
                       * t_delay) + t_offset) + 2.0F * t_acc) + nrev * t_rev)))
              {
                subcounter = counter - subcounter;
                t_offset = subcounter / f;
                subcounter = -0.5F * (omega_cruise * omega_cruise) / 12.5664F *
                  (t_offset * t_offset) + omega_cruise * subcounter / f;
                *x = x0 + R * cosf(subcounter);
                *y = b_y0 + R * sinf(subcounter);
                *z = z0 + z_target;

                /*  wait after the drone come to a stop */
              } else {
                if ((f * ((((t_takeoff + 2.0F * t_delay) + t_offset) + 2.0F *
                           t_acc) + nrev * t_rev) >= counter) || (counter > f *
                     ((((t_takeoff + 3.0F * t_delay) + t_offset) + 2.0F * t_acc)
                      + nrev * t_rev))) {
                  if ((f * ((((t_takeoff + 3.0F * t_delay) + t_offset) + 2.0F *
                             t_acc) + nrev * t_rev) < counter) && (counter <= f *
                       ((((t_takeoff + 3.0F * t_delay) + 2.0F * t_offset) + 2.0F
                         * t_acc) + nrev * t_rev))) {
                    *x = x0;
                    *y = b_y0 + R * (counter - f * (((t_takeoff + 3.0F * t_delay)
                      + t_offset) + t_acc)) / (f * t_offset);
                    *z = z0 + z_target;

                    /*  wait before landing */
                  } else {
                    if ((f * ((((t_takeoff + 3.0F * t_delay) + 2.0F * t_offset)
                               + 2.0F * t_acc) + nrev * t_rev) >= counter) ||
                        (counter > f * ((((t_takeoff + 4.0F * t_delay) + 2.0F *
                                          t_offset) + 2.0F * t_acc) + nrev *
                                        t_rev))) {
                      subcounter = f * ((((t_takeoff + 4.0F * t_delay) + 2.0F *
                                          t_offset) + 2.0F * t_acc) + nrev *
                                        t_rev);
                      if ((subcounter < counter) && (counter <= f * ((((2.0F *
                               t_takeoff + 4.0F * t_delay) + 2.0F * t_offset) +
                             2.0F * t_acc) + nrev * t_rev))) {
                        *x = x0;
                        *y = b_y0;
                        *z = (z_target - z_target / (f * t_takeoff) * (counter -
                               subcounter)) + z0;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

/*
 * File trailer for circl_traj.c
 *
 * [EOF]
 */
