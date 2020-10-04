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
#include <trajectory2.h>

/* this code defines a cross (X-pattern) trajectory, and the inputs are x, y and z of the vertices
3 0 2
0 C 0
1 0 4

pattern goes as follows:
-begin
- 1 to C
- C to 2 to C
- C to 3 to C
- C to 4 to C
- C to 1
- end
*/

/*x=[0 1 1 0]
y=[0 0 0 1]
z=[1 1 1 1]*/

float trajectory2(double x[4], double y[4], double z[4]){

	int n;
double dt = 0.005; //seconds, f=200Hz
double diag = pow(pow(x[2] - x[0],2) + pow(y[2] - y[0],2), 0.5);
double vcruise = 4.0 * diag/ (8000*dt);
double vx = vcruise * 0.707;
double vy = vx;
double xc = 0.5 * (x[2] + x[0]);
double yc = 0.5 * (y[2] + y[0]);

for (n=0;n<nmax2;n++){
	double tn=n*dt;
	loc2.zTraj[n]=1;

	if (n<floor(1*nmax2/8)){
		//moving from initial position (1 in diagram above) to C
		//step 1 on paper diagram
		double t=tn;
		loc2.xTraj[n] = x[0] + vx * t;
		loc2.yTraj[n] = y[0] + vy * t;
	}
	else if (n>=floor(1*nmax2/8) && n<floor(2*nmax2/8)){
		//moving from C to 2
		//steop 2 on paper
		double t = tn-floor(1*nmax2/8)*dt;	
		loc2.xTraj[n] = xc + vx * t;
		loc2.yTraj[n] = yc + vy * t;
	}
	else if (n>=floor(2*nmax2/8) && n<floor(3*nmax2/8)){
		//moving from 2 to C
		//step 3 on paper
		double t=tn-floor(2*nmax2/8)*dt;	
		loc2.xTraj[n] = x[2] - vx * t;
		loc2.yTraj[n] = y[2] - vy * t;
	}
	else if (n>=floor(3*nmax2/8) && n<floor(4*nmax2/8)){
		//moving from C to 3
		//step 4 on paper
		double t=tn-floor(3*nmax2/8)*dt;	
		loc2.xTraj[n] = xc + vx * t;
		loc2.yTraj[n] = yc - vy * t;
	}
	else if (n>=floor(4*nmax2/8) && n<floor(5*nmax2/8)){
		//moving from 3 to C
		//step 5 on paper
		double t=tn-floor(4*nmax2/8)*dt;	
		loc2.xTraj[n] = x[1] - vx * t;
		loc2.yTraj[n] = y[1] + vy * t;
	}
	else if (n>=floor(5*nmax2/8) && n<floor(6*nmax2/8)){
		//moving from C to 3
		//step 6 on paper
		double t=tn-floor(5*nmax2/8)*dt;	
		loc2.xTraj[n] = xc - vx * t;
		loc2.yTraj[n] = yc + vy * t;
	}
	else if (n>=floor(6*nmax2/8) && n<floor(7*nmax2/8)){
		//moving from 3 to 
		//step 7 on paper
		double t=tn-floor(6*nmax2/8)*dt;	
		loc2.xTraj[n] = x[3] + vx * t;
		loc2.yTraj[n] = y[3] - vy * t;
	}
	else if (n>=floor(7*nmax2/8) && n<floor(8*nmax2/8)){
		//moving from C to 1/initial position
		//step 8 on paper
		double t=tn-floor(7*nmax2/8)*dt;	
		loc2.xTraj[n] = xc - vx * t;
		loc2.yTraj[n] = yc - vy * t;
	}
}
return 0; 
}