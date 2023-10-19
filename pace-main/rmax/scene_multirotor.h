#ifndef rmax_scene_multirotor_h
#define rmax_scene_multirotor_h
#if defined(__cplusplus)
extern "C"
{
#endif


/* Copyright (c) Eric N. Johnson, 1998.  */

void initMultirotor( void );
void drawMultirotor( const double latitude,
		const double longitude,
		const double altitude,
        const double terrainAlt,
		const double eyeLat,
		const double eyeLon,
		const double eyeAlt,
		const double cosDatumLat,
		const double fovy,
		const float dcm[4][4],
		float sunPosition[4], int mode, int winh );

#if defined(__cplusplus)
}
#endif
#endif
