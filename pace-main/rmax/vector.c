

#include "vector.h"
#include <math.h>

double v3n(double a[3])
{
	return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

double v3dot(double a[3], double b[3])
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void vav(double a[3], double b[3], double vout[3])
{
	vout[0]=a[0]+b[0];
	vout[1]=a[1]+b[1];
	vout[2]=a[2]+b[2];
}

void vsv(double a[3], double b[3], double vout[3])
{
	vout[0]=a[0]-b[0];
	vout[1]=a[1]-b[1];
	vout[2]=a[2]-b[2];
}

void v3s(double a[3], double s, double vout[3])
{
	vout[0]=a[0]*s;
	vout[1]=a[1]*s;
	vout[2]=a[2]*s;
}

void v3cross(double a[3], double b[3], double vout[3])
{
	vout[0]=a[1]*b[2]-a[2]*b[1];
	vout[1]=a[2]*b[0]-a[0]*b[2];
	vout[2]=a[0]*b[1]-a[1]*b[0];
}

