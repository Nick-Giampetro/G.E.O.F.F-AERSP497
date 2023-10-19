#ifndef _RMAX_NOVATEL_H_
#define _RMAX_NOVATEL_H_

/**
 * Handles all interactions with NovAtel GPS.  This is not new code - this code
 *  is just moved from sensors.cpp to make the GPS code more modular.
 */

#if defined(__cplusplus)
extern "C"
{
#endif
/*
 * Checksum computation for NovAtel GPS
 * Defined here because used in gcs.c and si.c
 */
unsigned int gpsCheckSumCompute( unsigned char *buf, int byteCount );

/*
 * Update routines for the NovAtel, including configuration calls, etc.
 */
void updateNovatel(struct senNovatelGps_ref *novatel, unsigned char currentOutlierStatus, struct navinit_ref *init,
	struct obDatalink_ref *data, double time, char enableGPS, char safemode, char fltPlnManType,
	unsigned char isInCharge, unsigned char isPrimary);

/*
 * Note: I don't call these Novatel Out, because the Novatel structure is the same as used for several other
 *  GPS types that didn't need a customized one.  These two functions can be used for all of those.
 */
void outputGpsOut(struct gpsOut_ref *dest, struct gpsOut_ref *src, char registerOutliers);
void clearGpsOut(struct gpsOut_ref *src, unsigned char clearAll);
void timeOutGpsOut(struct gpsOut_ref *out, double timeout, double time);


#if defined(__cplusplus)
}
#endif

#endif // _RMAX_NOVATEL_H_