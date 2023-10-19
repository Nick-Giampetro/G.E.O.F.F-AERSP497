/*** BeginCopyright
 * Copyright 2002, Georgia Institute of Technology, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * UAV Laboratory
 * School of Aerospace Engineering
 * Georgia Institute of Technology
 * Atlanta, GA 30332
 * http://controls.ae.gatech.edu
 *
 * Contact Information:
 * Prof. Eric N. Johnson
 * http://www.ae.gatech.edu/~ejohnson
 * Tel : 404 385 2519
 * EndCopyright
 ***/
/***
 * $Id: navigation.cpp,v 1.205 2007-09-26 04:34:47 ejohnson Exp $
 * contains the navigation filter and also contains code to read sensors
 ***/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esim/util.h"
#include "rmax/onboard_ref.h"
#include "rmax/dted_ref.h"
#include "rmax/dted.h"
#include "rmax/navigation_ref.h"
#include "rmax/datalink.h"
#include "rmax/logger.h"

/* Prepares corresponding SRTM file if not opened */
static int srtmLoadTile( struct dted_ref *t, int latDec, int lonDec ) {

	if( t->latDec != latDec || t->lonDec != lonDec || t->fd == NULL ) {

		t->error = DTED_ERROR_NONE;
        t->latDec = latDec;
        t->lonDec = lonDec;

		sprintf( t->filename, "%s", "./dem/");

		//get filename of SRTM data
		if(                      latDec<=-10 ) {  strcat( t->filename, "S"  );
		} else if( latDec>-10 && latDec<0    ) {  strcat( t->filename, "S0" );
		} else if( latDec>=0  && latDec<10   ) {  strcat( t->filename, "N0" );
		} else {                                  strcat( t->filename, "N"  );
		}

		//sprintf( t->filename, "%s%d", t->filename, latDec );
		if( latDec >= 0 ) {
			sprintf( t->filename, "%s%d", t->filename, latDec );
		} else {
			sprintf( t->filename, "%s%d", t->filename, abs(latDec)+1 );
		}

		if(                       lonDec<=-100 ) {  strcat( t->filename, "W"   );
		} else if( lonDec>-100 && lonDec<=-10  ) {  strcat( t->filename, "W0"  );
		} else if( lonDec>-10  && lonDec<0     ) {  strcat( t->filename, "W00" );
		} else if( lonDec>=0   && lonDec<10    ) {  strcat( t->filename, "E00" );
		} else if( lonDec>10   && lonDec<100   ) {  strcat( t->filename, "E0"  );
		} else {                                    strcat( t->filename, "E"   );
		}

		if( lonDec >= 0 ) {
			sprintf( t->filename, "%s%d.hgt", t->filename, lonDec );
		} else {
			sprintf( t->filename, "%s%d.hgt", t->filename, abs(lonDec)+1 );
		}

        //sprintf(filename, "%s/%c%02d%c%03d.hgt", folder,
		//			latDec>0?'N':'S', abs(latDec),
		//			lonDec>0?'E':'W', abs(lonDec));

		t->fd = fopen( t->filename, "r");
        if( t->fd == NULL ) {
			t->error = DTED_ERROR_FILE;
			return -1; /* no file */
		} else {

			if( t->srtmTile == NULL ) {
				t->srtmTile = (unsigned char *) malloc( DTED_RTCM_SIZE ); //allocate only once
			}

			if( t->srtmTile != NULL ) {
				fread( (void *)(t->srtmTile), 1, DTED_RTCM_SIZE, (FILE *)(t->fd) );
				fclose( (FILE *)(t->fd) );
			} else {
				t->error = DTED_ERROR_MEMORY;
				fclose( (FILE *)(t->fd) );
				return -1; /* no memory */
			}
		}

    }

	return 0;

}


// Pixel idx from left bottom corner (0-1200)
static void srtmReadPx( struct dted_ref *t, int y, int x, int *height ) {

	int row = ( DTED_TOTALPX - 1 ) - y;
    int col = x;
    int pos = ( row*DTED_TOTALPX + col )*2;

    //set correct buff pointer
    unsigned char *buff = &(((unsigned char *)(t->srtmTile))[pos]);

    //solve endianity (using int16_t)
    short hgt = 0 | (buff[0] << 8) | (buff[1] << 0);

    if( hgt == -32768 ) {
		/*char buffer[200];
	    sprintf( buffer, "DTED: Void pixel (%d,%d)", x, y );
        logInfo( buffer );*/
		t->error = DTED_ERROR_DATA;
		*height = 0;
    } else {
		*height = (int) hgt;
	}

}


// Returns interpolated height from four nearest points
double srtmGetElevation( struct dted_ref *t, double lat, double lon ) {

    int latDec = (int)lat;
    int lonDec = (int)lon;

    if( -1 == srtmLoadTile( t, latDec, lonDec ) ) {
		return 0;
	}

	double heightM;
    double secondsLat = (lat-latDec)*3600;
    double secondsLon = (lon-lonDec)*3600;

	if( lonDec < 0 ) secondsLon += 3600;
	if( latDec < 0 ) secondsLat += 3600;

    //X coresponds to x/y values,
    //everything eastern/norhtern (< S) is rounded to X.
    //
    //  y   ^
    //  3   |       |   S
    //      +-------+-------
    //  0   |   X   |
    //      +-------+-------->
    // (sec)    0        3   x  (lon)

    //both values are 0-1199 (1200 reserved for interpolating)

    int y = (int)(secondsLat/DTED_SECONDSPERPIX);
    int x = (int)(secondsLon/DTED_SECONDSPERPIX);

    //get northern and eastern points
    int height[4];

    srtmReadPx( t, y,   x,   &height[2] );
    srtmReadPx( t, y+1, x,   &height[0] );
    srtmReadPx( t, y,   x+1, &height[3] );
    srtmReadPx( t, y+1, x+1, &height[1] );

    //ratio where X lays
    double dy = fmod( secondsLat, (double)DTED_SECONDSPERPIX )/DTED_SECONDSPERPIX;
    double dx = fmod( secondsLon, (double)DTED_SECONDSPERPIX )/DTED_SECONDSPERPIX;

    // Bilinear interpolation
    // h0------------h1
    // |
    // |--dx-- .
    // |       |
    // |      dy
    // |       |
    // h2------------h3
    heightM = height[0]*     dy *(1 - dx) +
              height[1]*     dy *     dx  +
              height[2]*(1 - dy)*(1 - dx) +
              height[3]*(1 - dy)*     dx;

	return heightM*C_M2FT;

}


void checkDEM( struct navigation_ref *n ) {

	struct navout_ref *o = n->out;
	struct dted_ref   *t = n->dted;

	switch( t->source ) {

	default: /* do nothing */
		break;

	case DTED_NONE:
		t->elevation = 0;
		break;

	case DTED_SRTM:
		t->elevation = srtmGetElevation( t, o->latitude, o->longitude ) - n->navinit->datumAlt;

		/* for testing purposes */
		if( 1 == t->checkMan ) { 
			t->manEle = srtmGetElevation( t, t->manLat, t->manLon ) - n->navinit->datumAlt;
		}

		break;

	}

	/* in case error happened on call above */
	if( t->error ) t->elevation = 0;

	/* allow manual adjustment for any reason */
	t->elevation += t->manualShift;

}






#if 0

 not sure about this stuff 


		/*
		FILE *filep1;
		char demfn[256];	// SRTM data file name

		n->dem->init = 0;
		n->dem->ok = 0;

		strcpy( demfn, "dem\\N33W089.hgt" );
		if( ( filep1 = fopen(demfn, "r+b" ) ) == NULL) {
		  printf( "Cannot open input file %s\n",demfn );
		  return;
		}

		printf( "nav: %s open success\n", demfn );
		n->dem->ok = 1;

		// Now to find out the number of records(double variables) in the realScene.bin file

		incr_ptr = j = 0;

		while((fread(&tp, sizeof(double), 1, filep1) == 1)) {
			incr_ptr++;
		}

		rewind( filep1 );

		// Now to call calloc() to perform DMA(Dynamic Memory Allocation) for records in file into an array

		ptr  = (double *)calloc(incr_ptr, sizeof(double));
		lat  = (double *)calloc(2, sizeof(double));
		lon  = (double *)calloc(5, sizeof(double));
		elev = (double *)calloc((incr_ptr-7), sizeof(double));

		//printf( "incr_ptr = %d, ptr address = %d", incr_ptr, ptr );

		for(i=0; i<incr_ptr; ++i) {
			fread(&ptr[i], sizeof(double), 1, filep1);
			if(i==1){
				lat[0] = ptr[i];
				//printf("\nLat[0] contains  = %lf", lat[0]);
			}

			if(i==(incr_ptr-4)){
				lat[1] = ptr[i];
				//printf("\nLat[1] contains  = %lf", lat[1]);
			}

			if(i>=2 && i<(incr_ptr-5)){
				elev[i-2] = ptr[i];
				//printf("\nElev[%d] contains  = %lf", i-2, elev[i-2]);
			}

			if(i==0){
				lon[0] = ptr[i];
				//printf("\nLon[0] contains  = %lf", lon[0]);
			}

			if(i==(incr_ptr-5)){
				lon[1] = ptr[i];
				//printf("\nLon[1] contains  = %lf", lon[1]);
			}

			if(i>=(incr_ptr-3)){
				if(j==0)
					lon_count = ptr[i];
					//lat_count = ptr[i];
				if(j==1)
					lat_count = ptr[i];
					//lon_count = ptr[i];
				if(j==2)
					total_count = ptr[i];

				j++;
			}

			//printf("\nptr[%d] contains  = %lf", i, ptr[i]);
		}

		printf("\nlat_count contains = %lf", lat_count);
		printf("\nlon_count contains = %lf", lon_count);
		printf("\ntotal_count contains = %lf", total_count);


		fclose( filep1 );

		// Initialize to enter the loop for checking lat and lon only once and then try to take estimates closer to it
		ent_loop = 0;
	*/
	}


	if( t->ok ) {
		if (0) {
			cur_lat = o->latitude;
			cur_lon = o->longitude;

			/*printf("\nCurrent lat = %lf, Current lon = %lf", cur_lat, cur_lon);*/

			/* Now to compare the current latitude and longitudes for obtaining the correct elevation from the array */

			arc_sec = 3./3600;
			lat_iter = lon_iter = 0;
			if( (ent_loop == 0) ||  (ent_loop == 1) ){
				el_lat = lat[0];
				el_lon = lon[0];
				if( ent_loop == 1 )
					ent_loop = 2;
			}

			/*printf("\nElev lat = %lf, Elev lon = %lf", el_lat, el_lon);*/

			for(i=0; i<(int)lon_count; i++) {

				if( el_lon < cur_lon ) {
					lon_iter = i;
					el_lon = el_lon + arc_sec;

				}
				else
				{
					lon_iter = i-1;
					el_lon = el_lon - arc_sec;
					i = (int)lon_count;
				}

			}

			/*printf("\nNearest lon = %lf", el_lon); */

			for(i=0; i<(int)lat_count; i++) {

				if( el_lat < cur_lat ) {
					lat_iter = i;
					el_lat = el_lat + arc_sec;

				}
				else
				{
					lat_iter = i-1;
					el_lat = el_lat - arc_sec;
					i = (int)lat_count;
				}

			}

			/*printf("\nNearest lat = %lf", el_lat);*/

			/* Formula from DEM data model saved in the realScene.bin (mcd) and realScene_mck.bin (mck) file for plotting appropriate elevations */

			el_val = (int)lat_count*lon_iter + lat_iter;
			nearest_el = elev[el_val];

			/*printf("\nNearest elevation = %lf", nearest_el);*/

			/* Now to perform 3 times 1-dimensional linear interpolation */

			/* using the linear interpolation formula as follows f(y_unkn) = [{((f(y2)-f(y1))/(y2-y1)} * (y_unkn-y1)] + f(y1) */

			/* First we do the latitude difference interpolation (longitude constant) */

			el1 = ((elev[el_val+1]-nearest_el)/arc_sec)*(cur_lat-el_lat) + nearest_el;

			/* Second we change the longitude and do latitude interpolation */

			el2 = ((elev[el_val+(int)lat_count+1]-elev[el_val+(int)lat_count])/arc_sec)*(cur_lat-el_lat) + elev[el_val+(int)lat_count];

			/* Third and final interpolation to obtain the final elevation from the previous two interpolations */
			/* Here the latitudes stay constant and longitudes change */

			t->elevation = (((el2 - el1)/arc_sec)*(cur_lon-el_lon) + el1)*C_M2FT - n->navinit->datumAlt;

			/*printf("\n first elevation is : %lf", el1*C_M2FT );
			printf("\n second elevation is : %lf", el2*C_M2FT );
			printf("\n final elevation is : %lf", n->dem->elevation+n->navinit->datumAlt );*/
			/*printf("\n final elevation is : %lf", n->dem->elevation );*/


#endif