#ifndef rmax_wdb_h
#define rmax_wdb_h

#if defined(__cplusplus)
extern "C"
{
#endif
void wdbInitDb(struct wdb_ref *wdb);
void wdbShutdownDb(struct wdb_ref *wdb);
void wdbInit();
double wdbGetMslAlt(struct wdb_ref *wdb, double x, double y);
double wdbGetRange( struct wdb_ref *wdb, double terrainAlt, double origin[], double dir[], double max, double angMax, double rangeSigma, double angSigma);

int intersect_triangle( double orig[3], double dir[3], double vert0[3], double vert1[3], double vert2[3], double *t, double *u, double *v, double *inc );

void command_wdbset(int argc, char **argv);
void command_wdbdatum(int argc, char **argv);
void command_wdbadd(int argc, char **argv);
void command_wdbclear(int argc, char **argv);
void command_wdbdatumned(int argc, char **argv);
void command_wdbmslalt(int argc, char **argv);
void command_wdbdatumob2(int argc, char **argv);
#if defined(__cplusplus)
}
#endif

#if defined(__cplusplus)
#include <math.h>
#include "esim/util.h"
inline void latlong2xy(double lat, double lon, double datumLat, double datumLon, double *px, double *py) {
	*px = ( lat - datumLat )*C_NM2FT*60.0;
	*py = ( lon - datumLon )*C_NM2FT*60.0*cos( lat*C_DEG2RAD );
}

inline void xy2latlong(double x, double y, double datumLat, double datumLon, double *lat, double *lon) {
	*lat  = datumLat + x/60.0*C_FT2NM;
	*lon  = datumLon + y/60.0*C_FT2NM/cos( *lat*C_DEG2RAD );
}

// is point in poly (i think only for convex polygons)
// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
inline int pnpoly(int npol, double *xp, double *yp, double x, double y)
{
	int i, j, c = 0;
	for (i = 0, j = npol-1; i < npol; j = i++) {
        if ((((yp[i]<=y) && (y<yp[j])) ||
			((yp[j]<=y) && (y<yp[i]))) &&
            (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))

			c = !c;
	}
	return c;
}
#endif



#endif
