#ifndef TRAJECTORY2
#define TRAJECTORY2

float trajectory2(double x[4], double y[4], double z[4]);


#define nmax2 8000

typedef struct loc_t2{
	float xTraj[nmax2];
	float yTraj[nmax2];
	float zTraj[nmax2];
} loc_t2;

loc_t2 loc2;

#endif // TRAJE:qCTORY
