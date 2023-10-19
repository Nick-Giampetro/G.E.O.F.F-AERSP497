#ifndef rmax_generic_h
#define rmax_generic_h
#if defined(__cplusplus)
extern "C"
{
#endif

/* Copyright (c) Eric N. Johnson, 1999.  */

void initGeneric( void );
void drawGeneric( const double latitude,
		  const double longitude,
		  const double altitude,
          const double terrainAlt,
		  const double eyeLat,
		  const double eyeLon,
		  const double eyeAlt,
		  const double cosDatumLat,
		  const double fovy,
		  const float dcm[4][4],
		  float sunPosition[4],
		  int mode, int winh, int rotated );


#if defined(__cplusplus)
}
#endif
#endif

