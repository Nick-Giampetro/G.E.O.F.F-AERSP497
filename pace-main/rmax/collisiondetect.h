#ifndef rmax_collisiondetect_h
#define rmax_collisiondetect_h

#if defined(__cplusplus)
extern "C"
{
#endif

double CollisionDetectLineVSTri(double position[3], double line[3], double vertex0[3], double vertex1[3], double vertex2[3]);
double CollisionDetectLineVSPara(double position[3], double line[3], double vertex0[3], double vertex1[3], double vertex2[3]);
void multiply3x3byVec(double mat[3][3], double vector[3]);
void addVecAndVec( double added[3], double original[3]);

#if defined(__cplusplus)
}
#endif

#endif
