#include "rmax/wdb_ref.h"
#include "esim/cnsl.h"
#include "esim/command.h"
#include "esim/db.h"
#include "esim/util.h"
#include "esim/rand.h"
#include "rmax/wdb.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rmax/navigation_ref.h"
#include "rmax/logger.h"

//typedef sulHashMap<sulString,sulString> Params;
static char cnslbuf[1024];

static struct wdb_ref *cwdb = 0;
static Dir *cwdbdir = 0;

void wdbInitDb(struct wdb_ref *wdb) {
}

void wdbShutdownDb(struct wdb_ref *wdb) {
}

//double wdbGetExtents(struct wdb_ref *wdb, double *xmin, double *xmax, double *ymin, double *ymax, double *zmin, double *zmax);

// http://www.cs.lth.se/home/Tomas_Akenine_Moller/raytri/raytri.c
/* Ray-Triangle Intersection Test Routines          */
/* Different optimizations of my and Ben Trumbore's */
/* code from journals of graphics tools (JGT)       */
/* http://www.acm.org/jgt/                          */
/* by Tomas Moller, May 2000                        */

/* code rewritten to do tests on the sign of the determinant */
/* the division is before the test of the sign of the det    */
/* and one CROSS has been moved out from the if-else if-else */

#define EPSILON 0.000001
#define CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define SUB(dest,v1,v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2];

int intersect_triangle( double orig[3], double dir[3],
			double vert0[3], double vert1[3], double vert2[3],
			double *t, double *u, double *v, double *inc )
{
   double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
   double det,inv_det;

   /* find vectors for two edges sharing vert0 */
   SUB(edge1, vert1, vert0);
   SUB(edge2, vert2, vert0);

   /* begin calculating determinant - also used to calculate U parameter */
   CROSS(pvec, dir, edge2);

   /* if determinant is near zero, ray lies in plane of triangle */
   det = DOT(edge1, pvec);

   /* calculate distance from vert0 to ray origin */
   SUB(tvec, orig, vert0);
   inv_det = 1.0 / det;

   CROSS(qvec, tvec, edge1);

   if (det > EPSILON)
   {
      *u = DOT(tvec, pvec);
      if (*u < 0.0 || *u > det)
	 return 0;

      /* calculate V parameter and test bounds */
      *v = DOT(dir, qvec);
      if (*v < 0.0 || *u + *v > det)
	 return 0;

   }
   else if(det < -EPSILON)
   {
      /* calculate U parameter and test bounds */
      *u = DOT(tvec, pvec);
      if (*u > 0.0 || *u < det)
	 return 0;

      /* calculate V parameter and test bounds */
      *v = DOT(dir, qvec) ;
      if (*v > 0.0 || *u + *v < det)
	 return 0;
   }
   else return 0;  /* ray is parallell to the plane of the triangle */

   *t = DOT(edge2, qvec) * inv_det;
   if (*t > EPSILON) {
	   double norm[3], mag;
       (*u) *= inv_det;
       (*v) *= inv_det;
	   /* get inclination */
	   CROSS( norm, edge1, edge2 );
	   mag = sqrt( SQ( norm[0] ) + SQ( norm[1] ) + SQ( norm[2] ) );
	   mag = DOT( norm, dir )/mag;
	   mag = ABS( mag );
	   (*inc) = acos( mag );
        return 1;
   } else {
        return 0;
   }
}


double wdbGetRange( struct wdb_ref *wdb, double terrainAlt, double origin[], double dir[], double max, double angMax, double rangeSigma, double angSigma) {

    double p0[3], p1[3], p2[3], p3[3];
    double t, u, v;

#if 0 //old code
    double a[3], b[3], tmp3[3], nplane[3], q[3], c[3], d[3];
    double tmp1,tmp2,tmp4,angle,angle1,angle2,angle3,angle4,dist, normdir, normnplane;
    double epsilon = 0.1;
    double an, bn, cn, dn;
#endif

    double minRange = 2*max;
    double inc = 0, newInc = 0;

    int i,j;

	/* check ground plane */
	if( dir[2] > 0 && origin[2] < 0 ) {
		inc = acos( dir[2] );
		minRange = ( -origin[2] - terrainAlt )/dir[2];
	}

    /* check wdb objects */
    for(i = 0; i < wdb->numQuads; i++) {
        for(j = 0; j<3; j++) {
            p0[j] = wdb->quads[i][0][j];
            p1[j] = wdb->quads[i][1][j];
            p2[j] = wdb->quads[i][2][j];
            p3[j] = wdb->quads[i][3][j];
        }

        if( intersect_triangle( origin, dir, p0, p2, p1, &t, &u, &v, &newInc ) ) { // check intersection with first triangle
            if (t<minRange) {
                minRange = t;
				inc = newInc;
            }
        } else if( intersect_triangle( origin, dir, p0, p3, p2, &t, &u, &v, &newInc ) ) { // check intersection with second triangle
            if (t<minRange) {
                minRange = t;
				inc = newInc;
            }
        }
    }

    if ((inc > angMax*C_DEG2RAD)||(minRange > max)) {
        return 0;
    } else {
        minRange += rangeSigma*randne()*minRange + angSigma*randne()*(inc*inc)/(angMax*angMax*C_DEG2RAD*C_DEG2RAD);
        return minRange;
    }

#if 0
	// Old code
    /* check wdb objects */
    for(i = 0; i < wdb->numQuads; i++) {
        for(j = 0; j<3; j++) {
            p1[j] = wdb->quads[i][0][j];
            p2[j] = wdb->quads[i][1][j];
            p3[j] = wdb->quads[i][2][j];
            p4[j] = wdb->quads[i][3][j];
            a[j] = p1[j]-p2[j];
            b[j] = p3[j]-p2[j];
            tmp3[j] = p1[j]-origin[j];
        }
        crossProduct3(nplane,a,b);
        norm3(normdir,dir);
        norm3(normnplane,nplane);
        dotProduct3(tmp1,nplane,dir);
        if (tmp1 != 0) {
            dotProduct3(tmp2,nplane,tmp3)
            tmp4 = tmp2/tmp1;
            if (tmp4 >=0) {
                for (j = 0; j < 3; j++) {
                    q[j] = origin[j] + tmp4*dir[j];
                    a[j] = p1[j]-q[j];
                    b[j] = p2[j]-q[j];
                    c[j] = p3[j]-q[j];
                    d[j] = p4[j]-q[j];
                }
                norm3(an,a);
                norm3(bn,b);
                norm3(cn,c);
                norm3(dn,d);
                angle1 = acos((a[0]*b[0]+a[1]*b[1]+a[2]*b[2])/(an*bn)); /* angle2lines(angle1,a,b); */
                angle2 = acos((c[0]*b[0]+c[1]*b[1]+c[2]*b[2])/(cn*bn)); /* angle2lines(angle2,b,c); */
                angle3 = acos((c[0]*d[0]+c[1]*d[1]+c[2]*d[2])/(cn*dn)); /* angle2lines(angle3,c,d); */
                angle4 = acos((a[0]*d[0]+a[1]*d[1]+a[2]*d[2])/(an*dn)); /* angle2lines(angle4,d,a); */
                angle = angle1 + angle2 + angle3 + angle4;
                if ((angle < (2*C_PI + epsilon)) && (angle > (2*C_PI - epsilon))) {
                    dist = sqrt((origin[0]-q[0])*(origin[0]-q[0]) + (origin[1]-q[1])*(origin[1]-q[1]) + (origin[2]-q[2])*(origin[2]-q[2]));
                    if (dist<minRange) {
                        minRange = dist;
                        inc = acos(-tmp1/(normdir*normnplane));
                    }
                }
            }
        }
    }
    if ((inc > angMax)||(minRange > max)) {
        return 0;
    } else {
        minRange += rangeSigma*randne()*minRange + angSigma*randne()*(inc*inc)/(angMax*angMax);
        return minRange;
    }
#endif
}

double wdbGetMslAlt(struct wdb_ref *wdb, double x, double y) {
	int i;
	double maxalt = wdb->datumAlt;
	double fpx[4];
	double fpy[4];

	struct wdbobj_ref *o;
	for(i = 0; i < wdb->numObjects; i++) {
		o = wdb->objs[i];
		if(o->type == WDB_OBJECT_BLDG) {

			double lsinpsi = o->length*sin(o->heading*C_DEG2RAD);
			double lcospsi = o->length*cos(o->heading*C_DEG2RAD);
			double wsinpsi = o->length*sin(o->heading*C_DEG2RAD);
			double wcospsi = o->length*cos(o->heading*C_DEG2RAD);
			latlong2xy(o->latitude, o->longitude, wdb->datumLat, wdb->datumLon, &fpx[0], &fpy[0]);
			fpx[1] = fpx[0] + lcospsi; fpy[1] = fpy[0] + lsinpsi;
			fpx[2] = fpx[1] - wsinpsi; fpy[2] = fpy[1] + wcospsi;
			fpx[3] = fpx[0] - wsinpsi; fpy[3] = fpy[0] + wcospsi;
			//fpx[4] = fpx[0];           fpy[4] = fpy[0];
			if(pnpoly(4, fpx, fpy, x, y)) {
				maxalt = MAX(maxalt,o->height+wdb->datumAlt);
			}
		} else if(o->type == WDB_OBJECT_HILL) {
			double xobj,yobj;
			latlong2xy(o->latitude, o->longitude, wdb->datumLat, wdb->datumLon, &xobj, &yobj);
			//double d = o->lamx*(xobj-x)*(xobj-x) + o->lamy*(yobj-y)*(yobj-y);
			//double alt = o->height*exp(-d/MAX(1,o->b));
			double d = (xobj-x)*(xobj-x)/(o->a*o->a) + (yobj-y)*(yobj-y)/(o->b*o->b);
			double alt = o->height*exp(-d/o->lam);

			maxalt = MAX(maxalt,alt+wdb->datumAlt);
		}
	}

	return maxalt;
}



void no_activedb() {
	logInfo("no active db, use \'wdbset gcswdb\' for example");
}

void command_wdbset(int argc, char **argv) {
	if(argc < 2) {
		logInfo("usage is wdbset <wdb dirname>");
		return;
	}

	Dir *dir = findDir(argv[1]);
	if(dir) {
		if(strcmp(dir->type,"wdb_ref")==0) {
			cwdb = (struct wdb_ref*)(dir->data);
			cwdbdir = dir;
			sprintf(cnslbuf,"wdb set to %s",cwdbdir->name);
			logInfo(cnslbuf);

		} else {
			sprintf(cnslbuf,"%s is not of type wdb_ref",dir->type);
			logInfo(cnslbuf);
			return;

		}
	} else {
		sprintf(cnslbuf,"%s not found",argv[1]);
		logInfo(cnslbuf);
		return;
	}

}

void command_wdbclear(int argc, char **argv) {
	if(cwdb) {
		cwdb->numObjects = 0;
		cwdb->numQuads = 0; // Added this line as sick laser still sees buildings after wdbclear command
		sprintf(cnslbuf,"cleared %s",cwdbdir->name);
		logInfo(cnslbuf);
	} else {
		no_activedb();
	}

}


void command_wdbdatum(int argc, char **argv) {
	char buf[1024];
	if(argc < 4) {
		logInfo("usage is wdbdatm latitude longitude msl-altitude");
		return;
	}
	if(cwdb) {
		cwdb->datumLat = atof(argv[1]);
		cwdb->datumLon = atof(argv[2]);
		cwdb->datumAlt = atof(argv[3]);
		sprintf(buf,"datum set to %f %f %f",cwdb->datumLat, cwdb->datumLon, cwdb->datumAlt);
		logInfo(buf);
	} else {
		no_activedb();
	}
}

void command_wdbdatumned(int argc, char **argv) {
	if(cwdb) {
		cwdb->datumLat = navinit.datumLat;
		cwdb->datumLon = navinit.datumLon;
		cwdb->datumAlt = navinit.datumAlt;
	} else {
		no_activedb();
	}
}

void command_wdbdatumob2(int argc, char **argv) {
	char buf[1024];
	if(cwdb) {
		sprintf(buf,"rc2 wdbdatum %f %f %f", cwdb->datumLat, cwdb->datumLon, cwdb->datumAlt);
		commandExecute(buf);
	} else {
		no_activedb();
	}

}

static int str2color(const char *color) {
	if(strcmp(color,"white") == 0) {
		return WDB_COLOR_WHITE;
	} else if(strcmp(color,"red") == 0) {
		return WDB_COLOR_RED;
	} else if (strcmp(color,"green") == 0) {
		return WDB_COLOR_GREEN;
	} else if (strcmp(color,"blue") == 0) {
		return WDB_COLOR_BLUE;
	} else if (strcmp(color,"yellow") == 0) {
		return WDB_COLOR_YELLOW;
	} else if (strcmp(color,"purple") == 0) {
		return WDB_COLOR_PURPLE;
	} else if (strcmp(color,"cyan") == 0) {
		return WDB_COLOR_CYAN;
	} else if (strcmp(color,"brown") == 0) {
		return WDB_COLOR_BROWN;
	} else if (strcmp(color,"pink") == 0) {
		return WDB_COLOR_PINK;
    } else if (strcmp(color,"orange") == 0) {
		return WDB_COLOR_ORANGE;
	} else if (strcmp(color,"gray") == 0) {
		return WDB_COLOR_GRAY;
	} else if (strcmp(color,"indigo") == 0) {
		return WDB_COLOR_INDIGO;
	} else {
		return WDB_COLOR_YELLOW;
	}
}

void wdbadd_usage( void ) {

               //0      1        2       3   4     5    6    7      8      9      10      11
	logInfo("Add objects using following template:");
	logInfo("wdbadd building latlong lat long  lt   alt  length width  height heading color");
	logInfo("wdbadd buildingtex latlong lat long  lt   alt  length width  height heading color");
	logInfo("wdbadd building ned     px  py    lt   alt  dx     dy     dz     heading");
	logInfo("wdbadd cylinder latlong lat long  msl  alt  radius height");
	logInfo("wdbadd tree     latlong lat long  msl  alt  height");
	logInfo("wdbadd hill     latlong lat long  lamx lamy b      height");
	logInfo("wdbadd wall     ned     px  py    lt   alt   length height heading");
	logInfo("wdbadd pole     ned     px  py    lt   alt   height radius");
	logInfo("wdbadd treeline latlong lat long msl alt lated longed height");
	logInfo("wdbadd water    latlong lat long  lt   alt  length width  heading flowspeed");

}

void command_wdbadd( int argc, char **argv ) { // Adds an object to the world database

	if( argc < 8 ) { // If user does not enter enough arguments to the console, send back an argument template
		wdbadd_usage();
		return;
	}

	if( cwdb ) {
		if( cwdb->numObjects >= WDB_MAX_OBJECTS ) {
			logWarning("Max number of db objects exceeded.");
			return;
		}

		struct wdbobj_ref *o = cwdb->objs[cwdb->numObjects];
		if(strcmp(argv[2],"latlong")== 0) {
			o->latitude  = atof(argv[3]);
			o->longitude = atof(argv[4]);
		} else if(strcmp(argv[2],"ned")==0) {
			xy2latlong(atof(argv[3]), atof(argv[4]), cwdb->datumLat, cwdb->datumLon, &(o->latitude), &(o->longitude));
		} else {
			sprintf(cnslbuf,"unknown coord specifier %s, should be [latlong,ned]",argv[2]);
			logWarning(cnslbuf);
		}
		if(strcmp(argv[5],"lt")) { // NOTE If argv[5] does not equal 'lt' --this is confusing!!
			o->altitude  = atof(argv[6]);
		} else if(strcmp(argv[5],"msl")) {// NOTE If argv[5] does not equal 'msl' --this is confusing!!
			o->altitude  = cwdb->datumAlt + atof(argv[6]);
		}

		if( (strcmp(argv[1],"building") == 0) || (strcmp(argv[1],"buildingtex") == 0) ) {
			if(argc < 11) {
				logWarning("not enough arguments to add building");
				return;
			} else {
                double x0 = ( o->latitude - cwdb->datumLat )*C_NM2FT*60.0;
                double y0 = ( o->longitude - cwdb->datumLon )*C_NM2FT*60.0*cos( o->latitude*C_DEG2RAD );
				if (strcmp(argv[1],"buildingtex") == 0) { o->type   = WDB_OBJECT_TEX_BLDG; }
				else { o->type   = WDB_OBJECT_BLDG; }
				o->length = atof(argv[7]);
				o->width  = atof(argv[8]);
				o->height = atof(argv[9]);
				o->heading = atof(argv[10]);
				if(argc > 11) {
					o->color = str2color(argv[11]);
				}
				cwdb->numObjects++;
                // front face
                cwdb->quads[cwdb->numQuads][0][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][1][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 + o->length*cos(o->heading*C_PI/180) - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->length*sin(o->heading*C_PI/180) + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 + o->length*cos(o->heading*C_PI/180) - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->length*sin(o->heading*C_PI/180) + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
                // top face
                cwdb->quads[cwdb->numQuads][0][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][1][0] = x0;
                cwdb->quads[cwdb->numQuads][1][1] = y0;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 + o->length*cos(o->heading*C_PI/180) - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->length*sin(o->heading*C_PI/180) + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude - o->height;
                cwdb->numQuads++;
                // back face
                cwdb->quads[cwdb->numQuads][0][0] = x0;
                cwdb->quads[cwdb->numQuads][0][1] = y0;
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0;
                cwdb->quads[cwdb->numQuads][1][1] = y0;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
                // bottom face
                cwdb->quads[cwdb->numQuads][0][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0;
                cwdb->quads[cwdb->numQuads][1][1] = y0;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][2][0] = x0 - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][3][0] = x0 + o->length*cos(o->heading*C_PI/180) - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->length*sin(o->heading*C_PI/180) + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
                // left face
                cwdb->quads[cwdb->numQuads][0][0] = x0;
                cwdb->quads[cwdb->numQuads][0][1] = y0;
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0;
                cwdb->quads[cwdb->numQuads][1][1] = y0;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
                // right face
                cwdb->quads[cwdb->numQuads][0][0] = x0 + o->length*cos(o->heading*C_PI/180) - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][1] = y0 + o->length*sin(o->heading*C_PI/180) + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0 + o->length*cos(o->heading*C_PI/180) - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][1][1] = y0 + o->length*sin(o->heading*C_PI/180) + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 - o->width*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->width*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
			}
		} else if( strcmp(argv[1],"water") == 0 ) {
			if(argc < 11) {
				logWarning("not enough arguments to add water");
				return;
			} else {
				o->type = WDB_OBJECT_WATER;
				o->length = atof(argv[7]);
				o->width  = atof(argv[8]);
				o->heading = atof(argv[9]);
				o->flowspeed = atof(argv[10]);
				cwdb->numObjects++;
			}
		} else if(strcmp(argv[1],"cylinder")==0) {
            o->type       = WDB_OBJECT_CYLINDER;
            o->radius     = atof(argv[7]);
            o->height     = atof(argv[8]);
            cwdb->numObjects++;
		} else if(strcmp(argv[1],"tree")==0) {
			o->type       = WDB_OBJECT_TREE;
			o->height     = atof(argv[7]);
			cwdb->numObjects++;
		} else if(strcmp(argv[1],"treeline")==0) {
			if(argc < 9) {
				logWarning("not enough arguments to add treeline");
				return;
			} else {
				o->type       = WDB_OBJECT_TREELINE;
				o->height     = atof(argv[9]);
				o->latitudeend =atof(argv[7]);
				o->longitudeend =atof(argv[8]);
				cwdb->numObjects++;
				/* represent treeline as a single quad */
                double x0 = ( o->latitude - cwdb->datumLat )*C_NM2FT*60.0;
                double y0 = ( o->longitude - cwdb->datumLon )*C_NM2FT*60.0*cos( o->latitude*C_DEG2RAD );
				double x1,y1,z1;
				x1 = ( o->latitudeend - cwdb->datumLat )*C_NM2FT*60.0;
				y1 = hmodDeg( o->longitudeend - cwdb->datumLon )*C_NM2FT*60.0*
				cos( o->latitudeend*C_DEG2RAD );
				z1 = /*wdb->datumAlt*/ - o->altitude;
				o->length = sqrt(pow(x0-x1,2)+pow(y0-y1,2));
				o->heading = atan2(y1-y0,x1-x0)*C_RAD2DEG;
                // single face
                cwdb->quads[cwdb->numQuads][0][0] = x0;
                cwdb->quads[cwdb->numQuads][0][1] = y0;
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0;
                cwdb->quads[cwdb->numQuads][1][1] = y0;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 + o->length*cos(o->heading*C_DEG2RAD);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->length*sin(o->heading*C_DEG2RAD);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 + o->length*cos(o->heading*C_DEG2RAD);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->length*sin(o->heading*C_DEG2RAD);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
			}
		} else if(strcmp(argv[1],"hill")==0) {
			o->type       = WDB_OBJECT_HILL;
			o->a          = atof(argv[5]);
			o->b          = atof(argv[6]);
			o->lam        = atof(argv[7]);
			o->height     = atof(argv[8]);
			o->a   = MAX(1, o->a);
			o->b   = MAX(1, o->b);
			o->lam = MAX(1,o->lam);
			o->height = MAX(WDB_HILL_MIN_ALT+1.0,o->height); // +1 makes sure height is always > min height
			// h = hmax*exp(-d/lam) where d = ((x-x0)/a)^2 + ((y-y0)/b)^2
			// say hin is min alt above which we consider to be inside the hill footprint
			// so,  d < -lam*ln(hin/hmax) to be inside footprint, say p^2 = -lam*lan(hin/hmax)
			// now lam > 0, and the ln() is < 0 if hin > hmax
			// hence ellipse within which hill exists is ((x-x0)/pa)^2 + ((y-y0)/pb)^2 = 1
			double psq = -o->lam*log(WDB_HILL_MIN_ALT/o->height);
			double pa = sqrt(psq*o->a*o->a);
			double pb = sqrt(psq*o->b*o->b);
			o->fpx[0] = pa;	// x value when y-y0 = 0
			o->fpy[0] = pb; // y value when x-x0 = 0
			cwdb->numObjects++;

		} else if(strcmp(argv[1],"wall")==0) {
		    if( argc < 10) {
                logWarning("not enough arguments to add wall");
				return;
		    } else {
                o->type       = WDB_OBJECT_WALL;
                o->length     = atof(argv[7]);
                o->height     = atof(argv[8]);
                o->width      = 1; /* this shouldn't be necessary? */
                o->heading    = atof(argv[9]);
                o->color      = WDB_COLOR_BROWN; /* good way to diff walls/fences */
                cwdb->numObjects++;
                /* represent wall as a single quad */
                double x0 = ( o->latitude - cwdb->datumLat )*C_NM2FT*60.0;
                double y0 = ( o->longitude - cwdb->datumLon )*C_NM2FT*60.0*cos( o->latitude*C_DEG2RAD );
                // single face
                cwdb->quads[cwdb->numQuads][0][0] = x0;
                cwdb->quads[cwdb->numQuads][0][1] = y0;
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0;
                cwdb->quads[cwdb->numQuads][1][1] = y0;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 + o->length*cos(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][1] = y0 + o->length*sin(o->heading*C_PI/180);
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
		    }

		} else if(strcmp(argv[1],"pole")==0) {
		    if( argc < 8) {
                logWarning("not enough arguments to add pole");
				return;
		    } else {
                o->type       = WDB_OBJECT_POLE;
                o->height     = atof(argv[7]);
                o->radius     = atof(argv[8]); /* pretend that telephone poles are 2ft diameter */
                cwdb->numObjects++;
                /* represent pole as two skinny intersecting planes */
                double x0 = ( o->latitude - cwdb->datumLat )*C_NM2FT*60.0;
                double y0 = ( o->longitude - cwdb->datumLon )*C_NM2FT*60.0*cos( o->latitude*C_DEG2RAD );
                // N-S quad
                cwdb->quads[cwdb->numQuads][0][0] = x0 + o->radius;
                cwdb->quads[cwdb->numQuads][0][1] = y0;
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0 + o->radius;
                cwdb->quads[cwdb->numQuads][1][1] = y0;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0 - o->radius;
                cwdb->quads[cwdb->numQuads][2][1] = y0;
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0 - o->radius;
                cwdb->quads[cwdb->numQuads][3][1] = y0;
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
                // E-W quad
                cwdb->quads[cwdb->numQuads][0][0] = x0;
                cwdb->quads[cwdb->numQuads][0][1] = y0 + o->radius;
                cwdb->quads[cwdb->numQuads][0][2] = o->altitude;
                cwdb->quads[cwdb->numQuads][1][0] = x0;
                cwdb->quads[cwdb->numQuads][1][1] = y0 + o->radius;
                cwdb->quads[cwdb->numQuads][1][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][2][0] = x0;
                cwdb->quads[cwdb->numQuads][2][1] = y0 - o->radius;
                cwdb->quads[cwdb->numQuads][2][2] = o->altitude - o->height;
                cwdb->quads[cwdb->numQuads][3][0] = x0;
                cwdb->quads[cwdb->numQuads][3][1] = y0 - o->radius;
                cwdb->quads[cwdb->numQuads][3][2] = o->altitude;
                cwdb->numQuads++;
		    }

		} else {
			sprintf(cnslbuf,"unkonwn object type %s",argv[1]);
			logWarning(cnslbuf);
		}
	} else {
		no_activedb();
	}

}


void command_wdbmslalt(int argc, char **argv) {
	if(argc < 3) {
		logInfo("usage is: wdbmslalt x y");
		return;
	}
	if(cwdb) {
		double mslalt = wdbGetMslAlt(cwdb, atof(argv[1]), atof(argv[2]));
		sprintf(cnslbuf,"%f",mslalt);
		logInfo(cnslbuf);
	} else {
		no_activedb();
	}

}
void wdbInit() {
	commandLoad("wdbset",      command_wdbset,      "sets active database");
	commandLoad("wdbdatum",    command_wdbdatum,    "sets datum of active database");

	commandLoad("wdbadd",      command_wdbadd,      "adds object to database");
	commandLoad("wdbclear",    command_wdbclear,    "clears active database");
	commandLoad("wdbdatumned", command_wdbdatumned, "sets dataum to current ned origin");
	commandLoad("wdbmslalt",   command_wdbmslalt,   "gets terrain altitude above msl");

	commandLoad("wdbdatumob2", command_wdbdatumob2, "copies current active database datum to ob2's active database");
}
