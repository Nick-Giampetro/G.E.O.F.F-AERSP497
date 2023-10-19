#ifndef VECTOR_H
#define VECTOR_H

#ifdef  __cplusplus
extern "C" {
#endif

double v3n(double a[3]);

double v3dot(double a[3], double b[3]);

void vav(double a[3], double b[3], double vout[3]);

void vsv(double a[3], double b[3], double vout[3]);

void v3s(double a[3], double s, double vout[3]);

void v3cross(double a[3], double b[3], double vout[3]);

#ifdef  __cplusplus
}
#endif

#endif