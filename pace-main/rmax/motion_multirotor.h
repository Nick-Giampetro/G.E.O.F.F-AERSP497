#ifndef rmax_motion_multirotor_h
#define rmax_motion_multirotor_h

#if defined(__cplusplus)
extern "C"
{
#endif

//#include "rmax/matrix.h"
/* Copyright (c) Eric N. Johnson, 1998.  */


void multirotor_f( struct vehicleSet_ref *set, struct vehicleMotion_ref *mo, struct state_ref *s, double dt );


#if defined(__cplusplus)
}
#endif

#endif

