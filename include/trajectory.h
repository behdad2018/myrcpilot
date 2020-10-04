#ifndef TRAJECTORY
#define TRAJECTORY

float trajectory(double x[4], double y[4], double z[4]);


#define nmax 1600

typedef struct loc_t{
	float xTraj[nmax];
	float yTraj[nmax];
	float zTraj[nmax];
} loc_t;

loc_t loc;

#endif // TRAJECTORY