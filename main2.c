/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "main.h"
#include "thrust.h"
#include "thrust_terminate.h"
#include <stdio.h>
#include "assert.h"
#include <time.h>
// char * means string

int main(int argc, const char * const argv[])
{
    assert(argc == 8);  // make sure we have 4 arguments
   
    float rpm_low = atof(argv[1]);   // atof string to number
    
    float rpm_high = atof(argv[2]);   // atof string to number

    float thrust_inp = atof(argv[3]);   // atof string to number
    
    float rho = atof(argv[4]);   // atof string to number
    
    float velocity[3];
    
//    1 2 3 is your speed, no [] and no ()
    
    velocity[0] = atof(argv[5]);
    velocity[1] = atof(argv[6]);
    velocity[2] = atof(argv[7]);
    
    
   printf("thrust is %f, density if %f and velocities are %f %f %f \n", thrust_inp ,rho, velocity[0],velocity[1],velocity[2]);
    // same for velocity
   
    clock_t start=clock();
    float rpm = thrust(rpm_low,rpm_high,thrust_inp,rho,velocity);
    clock_t end=clock();

   printf("rpm is %f\n and time is %f\n", rpm,(double)(end-start)/CLOCKS_PER_SEC);
    
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
//  main_thrust();

  /* Terminate the application.
     You do not need to do this more than one time. */
    thrust_terminate();
    
//  return 0;
}

/* End of code generation (main.c) */
