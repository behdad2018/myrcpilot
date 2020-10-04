#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/time.h>
#include <setpoint_manager.h>
#include <trajectory.h>

/* this code defines a square trajectory, and the inputs are x, y and z of the vertices
2 0 3
0 0 0
1 0 4

pattern goes as follows:
-begin
- 1 to 2
- 2 to 3
- 3 to 4
- 4 to 1
- end

every step consists of the following
-acceleration phase
-cruise phase
-deceleration phase
*/
/*x=[0 1 1 0]
y=[0 0 0 1]
z=[1 1 1 1]*/

float trajectory(double x[4], double y[4], double z[4]){

int n;
double dt = 0.005; //seconds, f=200Hz
// double acc = 0.02; for 16000 time steps
double acc = 2; //for 1600 time steps, agressive flight

double tacc = sqrt((abs(x[1] - x[0])+.000000001)/(2*acc)); 
double tdec = tacc; //seconds
double tcruise = 2 * tacc ; //compelete cruise time
double vcruise = (x[1] - x[0])/ (3 * tacc);
double dec = - acc; //m/s^s
// nmax= 16 * tacc / dt; 


// for hover, comment the top part and uncomment the following

// int n;
// double dt = 0.005; //seconds, f=200Hz
// double acc = 0.0;
// double tacc = 0.0; 
// double tdec = tacc; //seconds
// double tcruise = 2 * tacc ; //compelete cruise time
// double vcruise = 0.0;
// double dec = - acc; //m/s^s


for (n=0;n<nmax;n++){
	double tn=n*dt;
	loc.zTraj[n]=1;

	if (n<floor(nmax/16)){
		//accelerating from initial position
		double t=tn;
		loc.xTraj[n]= x[0] + 0.5 * acc * pow(t,2);
		loc.yTraj[n]= y[0];
	}
	else if (n>=floor(nmax/16) && n<floor(3*nmax/16)){
		//cruising towards setpoint 1
		double t = tn-floor(nmax/16)*dt;	
		loc.xTraj[n]= x[0]+ 0.5 * acc * pow(tacc,2) + vcruise * t;
		loc.yTraj[n]= y[0];
	}
	else if (n>=floor(3*nmax/16) && n<floor(4*nmax/16)){
		//decelerate into setpoint 1
		double t=tn-floor(3*nmax/16)*dt;	
		loc.xTraj[n]= x[0] + 0.5 * acc * pow(tacc,2) + vcruise * tcruise + 0.5 * dec * pow(t,2) + vcruise * t;
		loc.yTraj[n]= y[0];
	}
	else if (n>=floor(4*nmax/16) && n<floor(5*nmax/16)){
		//accelerating from setpoint 1 to 2
		double t=tn-floor(4*nmax/16)*dt;
		loc.xTraj[n]= x[1];	
		loc.yTraj[n]= y[1] + 0.5 * acc * pow(t,2) ;
	}
	else if (n>=floor(5*nmax/16) && n<floor(7*nmax/16)){
		//cruise from setooint 1 to 2
		double t=tn-floor(5*nmax/16)*dt;
		loc.xTraj[n] =  x[1];	
		loc.yTraj[n] =  y[1] + 0.5 * acc * pow(tacc,2) + vcruise * t ;
	}
	else if (n>=floor(7*nmax/16) && n<floor(8*nmax/16)){
		//decelerating into setpoint 2
		double t=tn-floor(7*nmax/16)*dt;
		loc.xTraj[n] =  x[1];	
		loc.yTraj[n] =  y[1] + 0.5 * acc * pow(tacc,2) + vcruise * tcruise  + 0.5 * dec * pow(t,2) + vcruise * t;
	}
	else if (n>=floor(8*nmax/16) && n<floor(9*nmax/16)){
		//accelerate out of point 2 moving towards setpoint 3
		double t=tn-floor(8*nmax/16)*dt;	
		loc.xTraj[n] = x[2] - 0.5 * acc * pow(t,2);
		loc.yTraj[n] = y[2];
	}
	else if (n>=floor(9*nmax/16) && n<floor(11*nmax/16)){
		//cruising from setpoint 2 towards setpoint 3
		double t=tn-floor(9*nmax/16)*dt;	
		loc.xTraj[n] = x[2] - 0.5 * acc * pow(tacc,2) - vcruise * t;
		loc.yTraj[n] = y[2];
	}
	else if (n>=floor(11*nmax/16) && n<floor(12*nmax/16)){
		//decelerating into setpoint 3
		double t=tn-floor(11*nmax/16)*dt;	
		loc.xTraj[n] = x[2] -  0.5 * acc * pow(tacc,2)  - vcruise * tcruise - 0.5 * dec * pow(t,2) - vcruise * t ;
		loc.yTraj[n] = y[2];
	}
	else if (n>=floor(12*nmax/16) && n<floor(13*nmax/16)){
		//accelerating from setpoint 3 towards setpoint 0
		double t=tn-floor(12*nmax/16)*dt;	
		loc.xTraj[n] = x[3];
		loc.yTraj[n] = y[3] - 0.5 * acc * pow(t,2) ;
	}
	else if (n>=floor(13*nmax/16) && n<floor(15*nmax/16)){
		//cruising from setpoint 3 to setpoint 0
		double t=tn-floor(13*nmax/16)*dt;
		loc.xTraj[n] = x[3];	
		loc.yTraj[n] = y[3] - 0.5 * acc * pow(tdec,2) - vcruise * t ;
	}
	else if (n>=floor(15*nmax/16) && n<floor(16*nmax/16)){
		//decelerating into setpoint 0
		double t=tn-floor(15*nmax/16)*dt;
		loc.xTraj[n]= x[3];	
		loc.yTraj[n]= y[3] - 0.5 * acc * pow(tacc,2) - vcruise * tcruise - 0.5 * dec * pow(t,2) - vcruise * t;
	}

}
return 0; 
}
