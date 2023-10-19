#ifndef rmax_dted_h
#define rmax_dted_h

#if defined(__cplusplus)
extern "C"
{
#endif


#include "rmax/dted_ref.h"
double srtmGetElevation( struct dted_ref *t, double lat, double lon );
void checkDEM( struct navigation_ref *n );


#if defined(__cplusplus)
}
#endif


#endif
