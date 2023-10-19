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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>   // for timer functions
#include <stdint.h>   // for timer functions
#include <sys/timeb.h>
#include <sys/types.h>

#include "esim/util.h"
#include "esim/quat.h"
#include "esim/rand.h"
#include "esim/sim_ref.h"
#include "esim/input_ref.h"

#include "rmax/matrix.h"
#include "rmax/motion_ref.h"
#include "rmax/serial.h"
#include "rmax/si.h"
#include "rmax/gcs.h"
#include "rmax/joyinputs_ref.h"
#include "rmax/onboard_ref.h" /* just for no actuators mode */
#include "rmax/navigation_ref.h" /* just for rmax input mode, platform estimates */
#include "rmax/controller_ref.h"
#include "rmax/sensors_ref.h"
#include "rmax/sensors.h"
#include "rmax/aglSensor.h"
#include "rmax/novatel.h"
#include "rmax/realScene_ref.h"  /* to include building data for LRF */
#include "rmax/collisiondetect.h"
#include "rmax/scene_ref.h"
#include "rmax/generic_ref.h"
#include "rmax/checksum.h"
#include "rmax/wdb.h"
#include "rmax/wdb_ref.h"
#include "rmax/vector.h"

#define IFLIBEXT(CODE)

FILE *datafile;

static void si_swab( char *from, char *to, int n ) {

	int i;
	char switchByte;

	for( i=0; i<n; i+=2 ) {
		switchByte = from[i];
		to[i]      = from[i+1];
		to[i+1]    = switchByte;
	}

}

unsigned char crc8_terra_SI(unsigned char *p, unsigned char len){
    unsigned int i;
    unsigned int crc = 0x0;
    while(len--){
        i = (crc ^ *p++) & 0xFF;
        crc = (crc_table_terra[i] ^ (crc << 8)) & 0xFF;
    }
    return crc & 0xFF;
}

/* starts the timer to regulate the acquisition*/
static void updatePlayback( struct si_ref *s ) {

#if 0
	/* this is no longer supported */


	struct playback_ref *pr = s->play;
	int var;
//	long itime = navout.itime;
	struct datarecord_ref *dr = &datarecord;
	double junk;
	int i;

	/* this section is currently configured to playback
	   raw sensor data to stimulate the nav system */

	if ( pr->on ) {

		if( pr->init ) {
			if( pr->valid ) {
				fclose( datafile );
				pr->valid = 0;
			}
			pr->init = 0;
		}

		if( !pr->valid ) {
			datafile = (void *)fopen( pr->fileName, "r+b");
			if( datafile != NULL ) {
				//rewind( datafile );
				//if ( feof(datafile) != 0 ) {
				pr->valid = 1;
				fread ( &pr->itime, 8, 1, datafile );
				//}
				pr->lastUpdate = state.time;
				pr->first = 2;
			} else
				pr->on = 0;
		}

		if( pr->valid ) if( state.time >= pr->lastUpdate + pr->updateDt )
			/*if( itime >= pr->itime )*/ {
			pr->lastUpdate += pr->updateDt;

			navout.init = 0;

			if ( pr->itime%dr->fh->frameSkip[0] == 0 ) {
				for ( var = 0; var < RECORD_VARS0; var++ ) {
					fread ( dr->record0[var], sizeof(double), 1, datafile );
				}
				if( pr->lastImu != imuOut.s_b_e_B[0] )
					imuOut.itime++;
				pr->lastImu = imuOut.s_b_e_B[0];

				if ( pr->itime%dr->fh->frameSkip[1] == 0 ) {
					for ( var = 0; var < RECORD_VARS1; var++ ) {
						fread ( &junk /*dr->recordDoubles1[var]*/, sizeof(double), 1, datafile );
						//fread ( dr->recordDoubles1[var], sizeof(double), 1, datafile );
					}

					if ( pr->itime%dr->fh->frameSkip[2] == 0 ) {
						for ( var = 0; var < RECORD_VARS2; var++ ) {
							fread ( dr->record2[var], sizeof(double), 1, datafile );
						}
						if( pr->lastGpsPos != gpsOut.p_b_e_L[0] ) {
							gpsOut.itimePos++;
							if( pr->first ) {
								for( i=0; i<3; i++ ) {
									navstate.x[i] = gpsOut.p_b_e_L[i];
									navstate.wb[i] = pr->wb[i];
									navstate.ab[i] = pr->ab[i];
								}
								//euler2quat( navstate.q, pr->phi*C_DEG2RAD, pr->theta*C_DEG2RAD, pr->psi*C_DEG2RAD );
								for( i=0; i<4; i++ )
									navstate.q[i] = pr->q[i];
								pr->first--;
							}
						}
						pr->lastGpsPos = gpsOut.p_b_e_L[0];

						if( pr->lastGpsVel != gpsOut.v_b_e_L[0] ) {
							gpsOut.itimeVel++;
							if( pr->first ) {
								for( i=0; i<3; i++ )
									navstate.v[i] = gpsOut.v_b_e_L[i];
								pr->first--;
							}
						}
						pr->lastGpsVel = gpsOut.v_b_e_L[0];

						if( pr->lastMagnet != magnetOut.field_B[0] )
							magnetOut.itime++;
						pr->lastMagnet = magnetOut.field_B[0];

						//if( pr->lastSonar != senSonarData.altitude )
						//	senSonarData.itime++;
						//pr->lastSonar = senSonarData.altitude;

						if ( pr->itime%dr->fh->frameSkip[3] == 0 ) {
							for ( var = 0; var < RECORD_VARS3; var++ ) {
								fread ( dr->record3[var], sizeof(double), 1, datafile );
							}
						}
					}
				}
			}

			fread ( &pr->itime, 8, 1, datafile );
		}
	}
#endif
}



static unsigned char heliCheckSumCompute( unsigned char *buf, int byteCount ) {

	int m;
	unsigned char csum;

	csum = 0;
	for( m=0; m<byteCount; m++ )
		csum ^= buf[m];

	return( csum );

}


static unsigned char imuCheckSumCompute( unsigned char *buf, int byteCount ) {

  int m;
  unsigned char csum;

  csum = 0;
  for( m=0; m<byteCount; m++ )
    csum ^= buf[m];

  return( csum );

}


/*static unsigned char gpsCheckSumCompute( unsigned char *buf, int byteCount ) {

  int m;
  unsigned char csum;

  csum = 0;
  for( m=0; m<byteCount; m++ )
    csum ^= buf[m];

  return( csum );

}*/


static void encodeYAS( unsigned char index, unsigned char *bf, short value ) {

	/* this just not send real data, it just get's the index correct */

	bf[0] = index<<4;
	bf[1] = 1<<7;

}


void gpsDate( struct gpsModel_ref *gps, double time ) {

	if( sim.mode == SIM_MODE_INIT ) {

		int numWeeks;
		double curJD;
		double rtcEpoch = 2440587.5; // computer realtime clock returns seconds since Jan 1 1970 00:00 This is the julian date for that day.
		double gpsEpoch = 2444244.5; // This is the julian date of for the date gps weeks started counting Jan 6 1980 00:00

		#if defined(LINUX) || defined(QNX)
			struct timespec OBtimer;
			clock_gettime(CLOCK_REALTIME, &OBtimer);    
			curJD = rtcEpoch + (OBtimer.tv_sec + OBtimer.tv_nsec/1e9 + gps->leapSeconds)/8.64e4;
		#elif defined(WIN32)
			struct _timeb OBtimer;
			_ftime(&OBtimer);
			curJD = rtcEpoch + (OBtimer.time + OBtimer.millitm/1000 + gps->leapSeconds)/8.64e4;
		#endif

		numWeeks = (int)((curJD - gpsEpoch)/7);
		gps->gpsMowStart = (long)((curJD - gpsEpoch - numWeeks * 7) * 8.64e7);
		gps->gpsWeek = numWeeks;
	   /* *gpsWeek = numWeeks%1024; */ /* novatell gives us the absolute week */
		gps->weekRollover = (int)(numWeeks/1024);

	} else {

		gps->gpsMow = (long)(gps->gpsMowStart + time*1000);

	}

}


static void encodeYRD( unsigned char index, unsigned char *bf, short value ) {

	value = ( value - 900 )*512/1200;

	bf[0] = ( index<<2 ) | ( value>>7 );
	bf[1] = ( 0x80 ) | ( value&0x7f );

}

static void rateLimit( double in, double *state, double dtOverLimit ) {

	double da;

	da =  in - *state;

	*state += LIMIT( da, -dtOverLimit, +dtOverLimit );

}

static void rateAccelLimit( double command, double *position, double *rate, double ratetime, double acceltime, double dt ) {

	double rateDes, errorsig;

	errorsig = command - *position;
	rateDes = ABS( errorsig )/dt;
	rateDes = MIN( rateDes, 1.0/ratetime );
	rateDes = MIN( rateDes, sqrt( 2/acceltime*ABS( errorsig ) ) );
	rateDes *= SIGN( errorsig );
	rateLimit( rateDes, rate, dt/acceltime );

	errorsig = command - *position - *rate*dt*0.5;
	rateDes = ABS( errorsig )/dt;
	rateDes = MIN( rateDes, 1.0/ratetime );
	rateDes = MIN( rateDes, sqrt( 2/acceltime*ABS( errorsig ) ) );
	rateDes *= SIGN( errorsig );
	rateLimit( rateDes, rate, dt/acceltime );
	*position += *rate*dt;

}

static void hysteresis( double command, double *position, double bound ) {

	if(   command > *position + bound ) {
		*position = command - bound;
	} if( command < *position - bound ) {
		*position = command + bound;
	}

}

void updateSIInputs( void ) {

	struct vehicle_ref        *v  = &vehicle;

	struct vehicleMotion_ref  *m  = v->motion;
	struct state_ref          *st = m->state;
    struct motionControls_ref *co = m->controls;

	struct si_ref *s = &si;

    struct actuatorModel_ref *act    = s->act;
#ifdef PSP
    struct pspSI_ref         *psp    = s->psp;
#endif

	struct serialPort_ref    *port;

    struct input_ref         *joy    = &esimInput;

	struct gcsInstance_ref   *gi     = gcsActiveInstance( &gcs );

	int index, done, newRmaxData = 0, newPWMData = 0, newMessageData = 0;
    unsigned char *bf;

	if( 0 == s->run ) return;

//	static int crmem = 0, mbmem = 0;

	// Set the SI controller input settings to match the current gcs settings.
	//  The only caveat - if the setInputTypes flag is set, then set the gcs settings from the SI settings; that way,
	//  all the vehicle models can set their input settings (backwards compatible) without needing to access the GCS
	//  database.
	if( co->setInputTypes == 1 )
	{
		gi->cntrlInput->mode = co->mode;
		gi->cntrlInput->joystickMode = co->joystickMode;
		// Now that the values have been set, turn off the flag
		co->setInputTypes = 0;
	}
	else
	{
		co->mode = gi->cntrlInput->mode;
		co->joystickMode = gi->cntrlInput->joystickMode;
	}

	if( sim.mode == SIM_MODE_INIT && v->run ) {

		act->lastUpdateModel = -act->updateDtCmds;
		co->rollStick       = 0;
		co->pitchStick      = 0;
		co->rudderPedal     = 0;
		co->collectiveLever = 0;
		co->throttleLever   = 0;

	}

	if( !v->run ) return;

  /* actuator model */
	/* read actuator commands from pwm */

  port = act->pPWM;
	readPort( port );
	if( port->dataSource != PORT_OFF ) {

		done = 0;
		index = 0;

		while( ( index <= port->bytesread - (int)sizeof( struct datalinkHeader_ref ) ) && !done ) {

			if( ( port->buffer[index]   == DATALINK_SYNC0 ) &&
				( port->buffer[index+1] == DATALINK_SYNC1 ) &&
				( port->buffer[index+2] == DATALINK_SYNC2 ) ) {

				bf = &(port->buffer[index]);

				memcpy( act->pwmToSI, bf, sizeof( struct datalinkHeader_ref ) );

				if( datalinkCheckSumCompute( bf, sizeof( struct datalinkHeader_ref ) - sizeof( int )*2 ) == act->pwmToSI->hcsum &&
					act->pwmToSI->messageSize >= sizeof( struct datalinkHeader_ref ) &&
					act->pwmToSI->messageSize < BUFFERSIZE ) {

					if( act->pwmToSI->messageSize + index <= port->bytesread ) {
						/* have read in the entire message */

						/*((struct datalinkHeader_ref *)bf)->hcsum = 0;*/
						if( datalinkCheckSumCompute( &bf[sizeof( struct datalinkHeader_ref )], act->pwmToSI->messageSize - sizeof( struct datalinkHeader_ref ) ) == act->pwmToSI->csum ) {

							switch( act->pwmToSI->messageID ) {

     						case DATALINK_MESSAGE_PWM:
								memcpy( act->pwmToSI, bf, sizeof( struct datalinkMessagePWM_ref ) );
								newPWMData = 1;
								break;

							default:
								/* uncrecognized type */
								break;
							}

						}
						index += act->pwmToSI->messageSize - 1;

					} else {
						index--;
						done = 1;
					}
				} else { /* header checksum is bad */
					index += sizeof( struct datalinkHeader_ref ) - 1;
				}
			}
			index++; /* start seq not found, go to next byte */

			if( index < 0 ) index = BUFFERSIZE - 1;
		}
		clearPort( port, index );
	}
		static int allow_change = 1;
		/* joystick */
		if( gi->cntrlInput->joySiMan->output && allow_change ) {
			gi->cntrlInput->mode = MOTIONINPUT_JOYSTICK;
			co->autopilot = !co->autopilot;
			allow_change = 0;
		} else {
			if( gi->cntrlInput->joySiMan->output == 0 )
                allow_change = 1;
		}
}

static int pointInPolygon( double vertex[][2], int numVertices, float x, float y )
{
    // For determining if the person target is inside limits
    // From http://www.engr.colostate.edu/~dga/dga/papers/point_in_polygon.pdf

    int i, inPolygon = 0; // Start with presumption that point is outside

    // Convert coordinates into actual x,y positions
    float vi[2], viplusone[2], r;

    float w = 0;

    for( i = 0; i < numVertices; i++ ) {
        vi[0] = (float)(vertex[i][0] - x);
        vi[1] = (float)(vertex[i][1] - y);

        viplusone[0] = (float)(vertex[i+1][0] - x);
        viplusone[1] = (float)(vertex[i+1][1] - y);

        if( vi[1]*viplusone[1] < 0 ) { // line segment crosses x axis
            r = vi[0] + ( vi[1]*( viplusone[0] - vi[0] ) )/( vi[1] - viplusone[1] );
            // r is the x-coordinate of the intersection of the line segment and the x-axis

            if( r > 0 ) { // line segment crosses positive x axis
                if( vi[1] < 0 ) w += 1.0;
                else            w -= 1.0;
            }

        } else if( vi[1] == 0 && vi[0] > 0 ) { // vi is on positive x-axis
            if( viplusone[1] > 0 ) w += 0.5;
            else                   w -= 0.5;
        } else if( viplusone[1] == 0 && viplusone[0] > 0 ) { // viplusone is on the positive x-axis
            if( vi[1] < 0 ) w += 0.5;
            else            w -= 0.5;
        }

    }

    // Only inside the polygon if winding number is not equal to zero
    if( w != 0 ) inPolygon = 1;

    return inPolygon;
}
#if defined(HAVE_IGRAPH)
int placeNode(double pN, double pE, double pD, double size, double visited, int forceNode) {
	//node iterator


	igraph_add_vertices(&g, 1, 0); //in case maxed	
	SETVAN(&g, "pN", ig_num_nodes, pN);	//set current "child" node x,y 
	SETVAN(&g, "pE", ig_num_nodes, pE);
	SETVAN(&g, "pD", ig_num_nodes, pD);
	SETVAN(&g, "visited", ig_num_nodes, visited);
	SETVAN(&g, "visibility", ig_num_nodes, size);
	//printf("NOEVA: +Adding node %i @ %f N, %f E\n", ig_num_nodes, pN, pE);		

	return ig_num_nodes++;
}
#endif
int checkForLOS (double pN1, double pE1, double pD1, double pN2, double pE2, double pD2) {
	//check whether or not 2 points have (mutual) LOS
	struct vehicle_ref			*v   = &vehicle;
	struct vehicleMotion_ref	*m   = v->motion;
	struct wdb_ref				*wdb     = &gcswdb;
	
	double viewerPos[3] = {pN2,pE2,pD2};
	double viewerLOSDir[3] = {0,0,0};
	double LOSrangeViewer2AC;
	double actualRangeViewer2AC;
	double maxViewRange;

	double dN, dE, dD;
	double maxRangeViewer = 20000;

	dN = pN1-pN2;
	dE = pE1-pE2;
	dD = pD1-pD2;

	actualRangeViewer2AC = sqrt(SQ(dN) + SQ(dE) + SQ(dD));
	maxViewRange = MIN(actualRangeViewer2AC, maxRangeViewer);
	viewerLOSDir[0] = dN/actualRangeViewer2AC;
	viewerLOSDir[1] = dE/actualRangeViewer2AC;
	viewerLOSDir[2] = dD/actualRangeViewer2AC;
	LOSrangeViewer2AC = wdbGetRange(wdb, m->env->terrainAlt, viewerPos, viewerLOSDir, maxViewRange, 900,  0, 0);
	
	if (LOSrangeViewer2AC < actualRangeViewer2AC && LOSrangeViewer2AC) {
		//LOS obstructed
		return 0;
	} else {
		//viewer has LOS
		return 1;
	}

	
}

void initNovatelModel(struct gpsModel_ref *gps) {
	int j;

	gps->lastUpdatePos = -gps->updateDtPos - gps->p_latency;
	gps->lastUpdateVel = -gps->updateDtVel - gps->v_latency;
	gps->lastUpdateHdg = -gps->updateDtHdg - gps->h_latency;
	gps->lastUpdateDelay = -gps->updateDtDelay;
	gps->posTimeDither = 0;
	gps->velTimeDither = 0;

	for( j=0; j<3; j++ ){
	    gps->p_error[j] = 0.0;
	}
}

void updateNovatelModel(struct gpsModel_ref *gps,double time, double p_b_e_L[3], double v_b_e_L[3], 
												 double w_b_e_B[3],double dcm_bl[3][3],double psi){
	
	struct vehicleSet_ref  *set = vehicle.set; //use for refLatitude refLongitude datumAlt
	int j;

	/* GPS, get time delay buffers figured out first */
	if( time >= gps->lastUpdateDelay + gps->updateDtDelay ) {

		double dr_L[3];
		double dv_B[3], dv_L[3];

		/* heading first */
		gps->heading = psi;  /* could really be fancier to allow various antenna locations */

		time_delay_in( sim.mode == SIM_MODE_INIT, gps->heading, gps->h_buffer, SI_HDG_LATENCY );

		/* velocity */
		for( j=0; j<3; j++ ) {
			gps->v_b_e_L[j] = v_b_e_L[j];
		}

		/* move from c.g. */
		dv_B[0] = w_b_e_B[1]*gps->r[2] - w_b_e_B[2]*gps->r[1];
		dv_B[1] = w_b_e_B[2]*gps->r[0] - w_b_e_B[0]*gps->r[2];
		dv_B[2] = w_b_e_B[0]*gps->r[1] - w_b_e_B[1]*gps->r[0];
		map_vector(dcm_bl, dv_B, dv_L );
		mat_add( gps->v_b_e_L, 3, 1, dv_L, gps->v_b_e_L );

		for( j=0; j<3; j++ ) {
			time_delay_in( sim.mode == SIM_MODE_INIT, gps->v_b_e_L[j], gps->v_buffer[j], SI_VEL_LATENCY );
		}

		/* position */

		/* model sensor */
		for( j=0; j<3; j++ ) {
			gps->p_b_e_L[j] = p_b_e_L[j];
		}

		/* move from c.g. */
        map_vector(dcm_bl, gps->r, dr_L );
		mat_add( gps->p_b_e_L, 3, 1, dr_L, gps->p_b_e_L );

	    /* add noise */
		for( j=0; j<3; j++ ) {
            gps->p_error[j] =
				MAX( 0, 1.0 - gps->updateDtDelay/gps->p_T )*gps->p_error[j]
				+ gps->updateDtDelay*gps->p_sigma[j]*randne();
            gps->p_b_e_L[j] += gps->p_error[j] + gps->p_bias[j];
        }

		for( j=0; j<3; j++ ) {
			time_delay_in( sim.mode == SIM_MODE_INIT, gps->p_b_e_L[j], gps->p_buffer[j], SI_POS_LATENCY );
		}

		gps->lastUpdateDelay += gps->updateDtDelay; 
	}

	/* GPS - Heading */
	if( time >= gps->lastUpdateHdg + gps->updateDtHdg && 1 == gps->doHeading ) {

		gps->heading_delayed = time_delay_out( gps->h_latency/gps->updateDtDelay, gps->h_buffer, SI_HDG_LATENCY );

		/* add noise */
        gps->hsend = gps->heading_delayed + gps->h_sigma*randne() + gps->h_bias;

		gps->lastUpdateHdg += gps->updateDtHdg; /* sloppy, but probably realistic */

		gpsDate( gps, time - gps->h_latency );

		gps->header->messageID     = MESSAGE_HEADING2B;
		gps->header->headerLength  = sizeof( struct gps_header_ref );
		gps->header->messageLength = sizeof( struct gps_heading2b_ref ) - gps->header->headerLength - 4;
		gps->header->week          = gps->gpsWeek;
        gps->header->milliseconds  = gps->gpsMow;

		gps->heading2b->heading = (float)(gps->hsend);

		if( sim.mode == SIM_MODE_RUN ) {
			memcpy( (char *)gps->heading2b, (char *)gps->header, sizeof( struct gps_header_ref ) );
			gps->heading2b->crc = gpsCheckSumCompute( (char *)gps->heading2b, sizeof( struct gps_heading2b_ref ) - 4 );
			writePort( gps->p, (char *)gps->heading2b, sizeof( struct gps_heading2b_ref ) );
		}
	}

	/* GPS - Velocity*/
	if( time >= gps->lastUpdateVel + gps->updateDtVel ) {

		for( j=0; j<3; j++ ) {
			gps->v_b_e_L_delayed[j] = time_delay_out( gps->v_latency/gps->updateDtDelay, gps->v_buffer[j], SI_VEL_LATENCY );
		}

        /* add noise */
		if( (!(rand()%MAX(gps->v_outlierChance,1))) && gps->v_outlierEnable ) { // outlier occur at probability of 1/chance
			/* simulate outlier */
			for( j=0; j<3; j++ ) {
				gps->vsend[j] = gps->v_b_e_L_delayed[j] + gps->v_outlier_sigma[j]*randne() + gps->v_bias[j];
			}
		} else {
			for( j=0; j<3; j++ ) {
				gps->vsend[j] = gps->v_b_e_L_delayed[j] + gps->v_sigma[j]*randne() + gps->v_bias[j];
			}
		}

		gps->lastUpdateVel += gps->updateDtVel; /* sloppy, but probably realistic */

        gpsDate( gps, time - gps->v_latency );

		gps->header->messageID     = MESSAGE_BESTVELB;
		gps->header->headerLength  = sizeof( struct gps_header_ref );
		gps->header->messageLength = sizeof( struct gps_bestvelb_ref ) - gps->header->headerLength - 4;
		gps->header->week          = gps->gpsWeek;
        gps->header->milliseconds  = gps->gpsMow;

		gps->bestvelb->HorizontalSpeed = sqrt( SQ( gps->vsend[0] ) + SQ( gps->vsend[1] ) )*C_FT2M;
		gps->bestvelb->Track = atan2( gps->vsend[1], gps->vsend[0] )*C_RAD2DEG;
		gps->bestvelb->VerticalSpeed = -gps->vsend[2]*C_FT2M;

		if( sim.mode == SIM_MODE_RUN ) {
			memcpy( (char *)gps->bestvelb, (char *)gps->header, sizeof( struct gps_header_ref ) );
			gps->bestvelb->crc = gpsCheckSumCompute( (char *)gps->bestvelb, sizeof( struct gps_bestvelb_ref ) - 4 );
			writePort( gps->p, (char *)gps->bestvelb, sizeof( struct gps_bestvelb_ref ) );
		}
	}

	/* GPS - Position*/
	if( time >= gps->lastUpdatePos + gps->updateDtPos ) {

		for( j=0; j<3; j++ ) {
			gps->p_b_e_L_delayed[j] = time_delay_out( gps->p_latency/gps->updateDtDelay, gps->p_buffer[j], SI_POS_LATENCY );
		}
		
		/* add outlier noise */
		/* how about making it so both velocity and position have an outlier at the same time? */
		if( (!(rand()%MAX(gps->p_outlierChance,1))) && gps->p_outlierEnable ) { // outlier occur at probability of 1/chance
			/* simulate outlier */
			for( j=0; j<3; j++ ) {
				gps->psend[j] = gps->p_b_e_L_delayed[j] + gps->p_outlier_sigma[j]*randne();
			}
		} else {
			for( j=0; j<3; j++ ) {
				gps->psend[j] = gps->p_b_e_L_delayed[j];
			}
		}

		gps->lastUpdatePos += gps->updateDtPos; /* sloppy, but probably realistic */

		gpsDate( gps, time - gps->p_latency );

		/* encode position message */
		gps->header->messageID     = MESSAGE_BESTPOSB;
		gps->header->headerLength  = sizeof( struct gps_header_ref );
		gps->header->messageLength = sizeof( struct gps_bestposb_ref )	- gps->header->headerLength - 4;
		gps->header->week          = gps->gpsWeek;
        gps->header->milliseconds  = gps->gpsMow;

		/* note: this model does not account for misplacement of the reference station */
		gps->bestposb->Latitude  = set->refLatitude  + gps->psend[0]*C_FT2NM/60;
		gps->bestposb->Longitude = hmodDeg( set->refLongitude + gps->psend[1]*C_FT2NM/60/cos( set->refLatitude*C_DEG2RAD ));
		gps->bestposb->Altitude  = ( set->datumAlt   - gps->psend[2] )*C_FT2M;
		if( sim.mode == SIM_MODE_RUN ) {
			memcpy( (char *)gps->bestposb, (char *)gps->header, sizeof( struct gps_header_ref ) );
			gps->bestposb->crc = gpsCheckSumCompute( (char *)gps->bestposb, sizeof( struct gps_bestposb_ref ) - 4 );
			writePort( gps->p, (char *)gps->bestposb, sizeof( struct gps_bestposb_ref ) );
		}
	}
}

void updateIMUModel(struct imuModel_ref* imu, double latitude, double w_b_e_B[3],double a_b_e_L[3],double wd_b_e_B[3],double dcm_lb[3][3],double time){
	int j;

	imu->gravity = imu->grav0*( 1 + imu->grav1*SQ( sin( latitude*C_DEG2RAD ) ) );

	for( j=0; j<3; j++ ) {
		imu->w_b_e_B[j] = w_b_e_B[j];
		imu->s_b_e_L[j] = a_b_e_L[j];
	}
	imu->s_b_e_L[2] -= imu->gravity; /* convert to spec force */
	map_vector( dcm_lb, imu->s_b_e_L, imu->s_b_e_B );

	/* move from c.g. */

	/* a = acm + wd x r + w x ( w x r ) */
	imu->s_b_e_B[0] += wd_b_e_B[1]*imu->r[2] - wd_b_e_B[2]*imu->r[1]
		+w_b_e_B[0]*w_b_e_B[1]*imu->r[1] +w_b_e_B[0]*w_b_e_B[2]*imu->r[2]
		- imu->r[0]*( SQ(w_b_e_B[1] ) + SQ(w_b_e_B[2] ) );
	imu->s_b_e_B[1] += wd_b_e_B[2]*imu->r[0] - wd_b_e_B[0]*imu->r[2]
		+w_b_e_B[1]*w_b_e_B[2]*imu->r[2] +w_b_e_B[1]*w_b_e_B[0]*imu->r[0]
		- imu->r[1]*( SQ(w_b_e_B[2] ) + SQ(w_b_e_B[0] ) );
	imu->s_b_e_B[2] += wd_b_e_B[0]*imu->r[1] - wd_b_e_B[1]*imu->r[0]
		+w_b_e_B[2]*w_b_e_B[0]*imu->r[0] +w_b_e_B[2]*w_b_e_B[1]*imu->r[1]
		- imu->r[2]*( SQ(w_b_e_B[0] ) + SQ(w_b_e_B[1] ) );

	/* add biases */

	mat_add( imu->w_b_e_B, 3, 1, imu->wb_b_e_B, imu->w_b_e_B );
	mat_add( imu->s_b_e_B, 3, 1, imu->sb_b_e_B, imu->s_b_e_B );

	/* add white noise and shake */

	for( j=0; j<3; j++ ) {
		imu->w_b_e_B[j] += imu->w_sigma[j]*randne();
		imu->s_b_e_B[j] += imu->s_sigma[j]*randne();
		imu->w_b_e_B[j] += imu->w_shake[j] *sin( time*imu->omega_shake  + imu->phase_w_shake[j]  );
		imu->s_b_e_B[j] += imu->s_shake[j] *sin( time*imu->omega_shake  + imu->phase_s_shake[j]  );
		imu->w_b_e_B[j] += imu->w_shake2[j]*sin( time*imu->omega_shake2 + imu->phase_w_shake2[j] );
		imu->s_b_e_B[j] += imu->s_shake2[j]*sin( time*imu->omega_shake2 + imu->phase_s_shake2[j] );
	}

	/* apply saturation */

	for( j=0; j<3; j++ ) {
		imu->w_b_e_B[j] = LIMIT( imu->w_b_e_B[j], -imu->w_max, imu->w_max );
		imu->s_b_e_B[j] = LIMIT( imu->s_b_e_B[j], -imu->s_max, imu->s_max );
	}

	/* rotate */

	mat_transpose( (double *)imu->dcm_sb, 3, 3, (double *)imu->dcm_bs );
	map_vector( imu->dcm_bs, imu->w_b_e_B, imu->w_b_e_S );
	map_vector( imu->dcm_bs, imu->s_b_e_B, imu->s_b_e_S );
}

//main function for emulating sensors inlcuding IMU, GPS, FCS20 embedded sensors, PTZ, laser range finder and others
void updateSI( void ) {

	struct vehicle_ref        *v   = &vehicle;
	struct vehicleOutputs_ref *o   = v->outputs;
    struct vehicleSet_ref     *set = v->set;
	struct vehicleMotion_ref  *m   = v->motion;
    struct motionControls_ref *co  = m->controls;
	struct motionXforms_ref   *x   = m->xforms;
	struct state_ref          *st  = m->state;
	struct state_ref          *sd  = m->stateDot;
	struct env_ref            *env = m->env;
	double engineOmega = 0.0;
	double motor1Omega = 0.0;
	double motor2Omega = 0.0;

	struct si_ref *s = &si;

	struct wdb_ref          *wdb     = &gcswdb;

	struct imuModel_ref      *imu    = s->imu;
	struct imuMessage_ref    *im     = imu->m;

	struct magnetModel_ref   *magnet = s->magnet;
	struct magnetMessage_ref *mm     = magnet->m;

	struct sonarModel_ref    *sonar  = s->sonar;
	struct sonarMessage_ref  *sm     = sonar->m;

	struct gpsModel_ref      *gps    = s->gps;
    struct gps_satMeasurement_ref *prMeasurements = (struct gps_satMeasurement_ref*)(gps->range->measurements);

	struct actuatorModel_ref *act    = s->act;
	struct rmaxYCSraw_ref    *yr     = act->ycsRaw;

	/* we need the datalinkMessage to be able to
	   send to the onboard */

	struct gcs_ref           *g        = &gcs;
	struct gcsInstance_ref   *gi       = gcsActiveInstance( g );
	struct gcsDatalink_ref   *data     = gi->datalink;
	int j;
    
	//for terraranger laser alt
    unsigned char crc_terra;
    char buff_terra[4]={0};
    short terra_alt = 0;
		char terra_test[2] = { 0 };

	if( 0 == s->run ) return;

	updatePlayback( s );

	if( !v->run ) return;

	if( sim.mode == SIM_MODE_INIT ) {

	    s->lastUpdate = st->time;
		s->dt = 0;

		imu->lastUpdate = -imu->updateDt;
		im->count = 0;

		magnet->lastUpdate = -magnet->updateDt;

		sonar->lastUpdate = -sonar->updateDt;

		initNovatelModel(gps);

		act->lastUpdateYCS = -act->updateDtYCS;
		act->lastUpdateYAS = -act->updateDtYAS;
		act->lastUpdateYRD = -act->updateDtYRD;
		act->lastUpdatePWM = -act->updateDtPWM;
		act->lastUpdateSVI2AP = -act->updateDtSVI2AP;

	} else {

		s->dt = st->time - s->lastUpdate;
		s->lastUpdate = st->time;

	}

	/* inertial measurement unit */

	if( ( st->time >= imu->lastUpdate + imu->updateDt ) &&
		( sim.mode == SIM_MODE_RUN ) ) {

		updateIMUModel(imu,o->latitude,st->w_b_e_B,sd->v_b_e_L,sd->w_b_e_B,x->dcm_lb,st->time);
		//I moved to the code below to updateIMUModel function. Old code is kept just in case
		//Perhaps this is a better way to do it. It clearly separates modeling code from messaging code
		//For example, code for modeling ardu IMU is gonna be the same as modeling ISIS IMU but messaging will be different
		//so perhaps in the future you can delete modeling part of Ardu IMU and just replace it with the function above 
#if 0 
		/* model sensor */

		imu->gravity = imu->grav0*( 1 + imu->grav1*SQ( sin( o->latitude*C_DEG2RAD ) ) );

		for( j=0; j<3; j++ ) {
			imu->w_b_e_B[j] = st->w_b_e_B[j];
			imu->s_b_e_L[j] = sd->v_b_e_L[j];
		}
		imu->s_b_e_L[2] -= imu->gravity; /* convert to spec force */
		map_vector( x->dcm_lb, imu->s_b_e_L, imu->s_b_e_B );

		/* move from c.g. */

		/* a = acm + wd x r + w x ( w x r ) */
		imu->s_b_e_B[0] += sd->w_b_e_B[1]*imu->r[2] - sd->w_b_e_B[2]*imu->r[1]
			+ st->w_b_e_B[0]*st->w_b_e_B[1]*imu->r[1] + st->w_b_e_B[0]*st->w_b_e_B[2]*imu->r[2]
			- imu->r[0]*( SQ( st->w_b_e_B[1] ) + SQ( st->w_b_e_B[2] ) );
		imu->s_b_e_B[1] += sd->w_b_e_B[2]*imu->r[0] - sd->w_b_e_B[0]*imu->r[2]
			+ st->w_b_e_B[1]*st->w_b_e_B[2]*imu->r[2] + st->w_b_e_B[1]*st->w_b_e_B[0]*imu->r[0]
			- imu->r[1]*( SQ( st->w_b_e_B[2] ) + SQ( st->w_b_e_B[0] ) );
		imu->s_b_e_B[2] += sd->w_b_e_B[0]*imu->r[1] - sd->w_b_e_B[1]*imu->r[0]
			+ st->w_b_e_B[2]*st->w_b_e_B[0]*imu->r[0] + st->w_b_e_B[2]*st->w_b_e_B[1]*imu->r[1]
			- imu->r[2]*( SQ( st->w_b_e_B[0] ) + SQ( st->w_b_e_B[1] ) );

		/* add biases */

		mat_add( imu->w_b_e_B, 3, 1, imu->wb_b_e_B, imu->w_b_e_B );
		mat_add( imu->s_b_e_B, 3, 1, imu->sb_b_e_B, imu->s_b_e_B );

		/* add white noise and shake */

		for( j=0; j<3; j++ ) {
			imu->w_b_e_B[j] += imu->w_sigma[j]*randne();
			imu->s_b_e_B[j] += imu->s_sigma[j]*randne();
			imu->w_b_e_B[j] += imu->w_shake[j] *sin( st->time*imu->omega_shake  + imu->phase_w_shake[j]  );
			imu->s_b_e_B[j] += imu->s_shake[j] *sin( st->time*imu->omega_shake  + imu->phase_s_shake[j]  );
			imu->w_b_e_B[j] += imu->w_shake2[j]*sin( st->time*imu->omega_shake2 + imu->phase_w_shake2[j] );
			imu->s_b_e_B[j] += imu->s_shake2[j]*sin( st->time*imu->omega_shake2 + imu->phase_s_shake2[j] );
		}

		/* apply saturation */

		for( j=0; j<3; j++ ) {
			imu->w_b_e_B[j] = LIMIT( imu->w_b_e_B[j], -imu->w_max, imu->w_max );
			imu->s_b_e_B[j] = LIMIT( imu->s_b_e_B[j], -imu->s_max, imu->s_max );
		}

		/* rotate */

        mat_transpose( (double *)imu->dcm_sb, 3, 3, (double *)imu->dcm_bs );
		map_vector( imu->dcm_bs, imu->w_b_e_B, imu->w_b_e_S );
		map_vector( imu->dcm_bs, imu->s_b_e_B, imu->s_b_e_S );
#endif
		/* encode message */

		im->rate[0]  = (short)(imu->w_b_e_S[0]/imu->sfr); 
		im->rate[1]  = (short)(imu->w_b_e_S[1]/imu->sfr);
		im->rate[2]  = (short)(imu->w_b_e_S[2]/imu->sfr);
		if(imu->emulateADIS == 1)
		{
			// The ADIS IMU inverts the accelerometers.  Gyros remain as normal.  Hence, this flag
			im->accel[0] = (short)(-imu->s_b_e_S[0]/imu->sfa);
			im->accel[1] = (short)(-imu->s_b_e_S[1]/imu->sfa);
			im->accel[2] = (short)(-imu->s_b_e_S[2]/imu->sfa);
		}
		else
		{
			im->accel[0] = (short)(imu->s_b_e_S[0]/imu->sfa);
			im->accel[1] = (short)(imu->s_b_e_S[1]/imu->sfa);
			im->accel[2] = (short)(imu->s_b_e_S[2]/imu->sfa);
		}
		im->csum = imuCheckSumCompute( (unsigned char *)im, sizeof( struct imuMessage_ref )-1 );

		/* send message */

		writePort( imu->p, (char *)im, sizeof( struct imuMessage_ref ) );

		/* deal with counters */

		im->count++;
		if( im->count == 100 )
			im->count = 0;

		imu->lastUpdate += imu->updateDt; /* sloppy, but probably more realistic */

	}

	//Novatel GPS emulation is moved to updateNovatelModel() function
	//Old code is kept just in case..
	updateNovatelModel(gps, st->time, st->p_b_e_L, st->v_b_e_L, st->w_b_e_B, x->dcm_bl, o->psi);

	/* magnetometer */

	if( ( st->time >= magnet->lastUpdate + magnet->updateDt ) &&
		( sim.mode == SIM_MODE_RUN ) ) {

		double clat, slat, clon, slon; //, mag;

		/* find magnet field line */

		clat = cos( o->latitude*C_DEG2RAD );
		slat = sin( o->latitude*C_DEG2RAD );
		clon = cos( o->longitude*C_DEG2RAD );
		slon = sin( o->longitude*C_DEG2RAD );
		magnet->field_L[0] = -( magnet->g10*clat - ( magnet->g11*clon + magnet->h11*slon )*slat );
		magnet->field_L[1] =                       ( magnet->g11*slon - magnet->h11*clon );
		magnet->field_L[2] = -( magnet->g10*slat + ( magnet->g11*clon + magnet->h11*slon )*clat )*2;
		//mag = sqrt( SQ( magnet->field_L[0] )
		//	+ SQ( magnet->field_L[1] ) + SQ( magnet->field_L[2] ) );
		for( j=0; j<3; j++ )
			magnet->field_L[j] *= magnet->field_mag;

		/* model sensor */

		map_vector( x->dcm_lb, magnet->field_L, magnet->field_B );

        /* add white noise */

		for( j=0; j<3; j++ )
            magnet->field_B[j] += magnet->field_sigma*randne() + magnet->err_B[j];

		/* rotate */
        
        // rotate around body z axis (for debugging a mounting error )
		{
			double dcm_bb2[3][3] = {{0}};
			double field_B2[3] = {0};
			euler2dcm(0, 0, magnet->psi_e, dcm_bb2);
			map_vector( dcm_bb2, magnet->field_B, field_B2 );

			mat_transpose( (double *)magnet->dcm_sb, 3, 3, (double *)magnet->dcm_bs );
			map_vector( magnet->dcm_bs, field_B2, magnet->field_S );
		}
        
		/* encode and send message */
		switch (magnetSet.magnetType){
			case MAGTYPE_RMAX:
			default:
				for( j=0; j<3; j++ ) {
					mm->values[j]  = (short)(magnet->field_S[j]);
				}
				si_swab( (char *)mm, (char *)mm, sizeof( struct magnetMessage_ref )-1 );
				writePort( magnet->p, (char *)mm, sizeof( struct magnetMessage_ref ) );
				break;
			case MAGTYPE_HMR3400:
				{
					char buffer[40];
					sprintf( buffer, "%d,%d,%d%c%c", mm->values[0], mm->values[1], mm->values[2], 0x0D, 0x0A );
					writePort( magnet->p, buffer, strlen( buffer ) );
				}
				break;
		}

		/* deal with counters */

		magnet->lastUpdate += magnet->updateDt; /* sloppy, but probably more realistic */

	}

/* sonar */

	if( ( st->time >= sonar->lastUpdate + sonar->updateDt ) &&
		( sim.mode == SIM_MODE_RUN ) ) {

		/* defines variables for location, direction of sonar and temp variable*/
		double r_L[3], dir_B[3], dir_L[3], alt_temp_sonar;
		double sonar_cone; // half angle of cone in radians

		sonar_cone = C_DEG2RAD*sonar->fov*0.5;

		/* model sensor */
		map_vector( x->dcm_bl, sonar->r, r_L );
        for( j=0; j<3; j++ ) {
            r_L[j] += st->p_b_e_L[j];
        }

		/* initialize measurement */
		sonar->altitude = sonar->alt_max;
		/* Creates a cone of rays, and returns the shortest one */
		/* I fake it a bit by making the cone go straight down */
		for( j=0; j<9; j++ ) {

			/* get direction */
			if( j == 0 ) {
				if( sonar->pointsX ) {
					dir_B[0] = 1;
					dir_B[1] = 0;
					dir_B[2] = 0;
				} else {
					dir_B[0] = 0;
					dir_B[1] = 0;
					dir_B[2] = 1;
				}
			} else {
				if( sonar->pointsX ) {
					dir_B[0] = cos(sonar_cone);
					dir_B[1] = sin(sonar_cone)*sin( (j-1)*C_PI/4 );
					dir_B[2] = sin(sonar_cone)*cos( (j-1)*C_PI/4 );
				} else {
					dir_B[0] = sin(sonar_cone)*cos( (j-1)*C_PI/4 );
					dir_B[1] = sin(sonar_cone)*sin( (j-1)*C_PI/4 );
					dir_B[2] = cos(sonar_cone);
				}
			}
			map_vector( x->dcm_bl, dir_B, dir_L );

			/* truth distance */
			alt_temp_sonar = wdbGetRange( wdb, m->env->terrainAlt, r_L, dir_L, sonar->alt_max, sonar->incidence_max, 0, 0 );
			if( alt_temp_sonar == 0 ) alt_temp_sonar = sonar->alt_max;

			/* saturation */
			alt_temp_sonar = LIMIT( alt_temp_sonar, sonar->alt_min, sonar->alt_max );

			/* compare with previous minimum*/
			if ( alt_temp_sonar < sonar->altitude) sonar->altitude = alt_temp_sonar;
		}

		/* add white noise and error */
        sonar->alt_error = sonar->alt_sigma*randne()*sonar->altitude + sonar->alt_bias;
		sonar->altitude += sonar->alt_error;
		sonar->altitude = LIMIT( sonar->altitude, sonar->alt_min, sonar->alt_max );

		/* simulate outlier */
		if( sonar->outlierEnable ) {
			if( sonar->outlierInProgress > 0 ) {
				if( (!(rand()%MAX(sonar->chanceRecover,1))) ) { // outlier occur at probability of 1/chance
					sonar->outlierInProgress = 0;
				} else {
					if( sonar->outlierInProgress < sonar->altitude ) { 
						sonar->altitude = sonar->outlierInProgress;
					} else {
						sonar->outlierInProgress = 0; /* catch the case where the outlier is now bigger than the real value */
					}
				}
			} else {
				if( (!(rand()%MAX(sonar->chance,1))) ) { // outlier occur at probability of 1/chance
					sonar->altitude = sonar->alt_min + rande()*( sonar->altitude - sonar->alt_min );
					sonar->outlierInProgress = sonar->altitude;
				}
			}
		} else {
			sonar->outlierInProgress = 0;
		}

		/* encode message */
		sonar->raw = (int)(sonar->altitude/sonar->sf + sonar->bias);

		//send message
		switch (sonar->encodeMode){ //encode different messages for each hardware
		default:
			    if( sonar->error ) {
	    		    if( sonar->altitude == sonar->alt_min || sonar->altitude == sonar->alt_max )
		    		    sonar->raw = sonar->error;
		         }
		        sprintf( sm->data, "%04X\r", LIMIT( sonar->raw, 0, 0xffff ) );
		        break;

		case SI_SONAR_QUAD:
		        sprintf( sm->data, "R%03d\r", LIMIT( sonar->raw, 0, 999 ) );
		        break;
        case SI_SONAR_TERRA:
            alt_temp_sonar = wdbGetRange( wdb, m->env->terrainAlt, r_L, dir_L, sonar->alt_max, sonar->incidence_max, 0, 0 );
            terra_alt = (short)(alt_temp_sonar*C_FT2M*C_M2MM); //convert from ft to mm

            if(terra_alt > 14000 || terra_alt < 200){ //max and min are 14000mm and 200mm
                terra_test[0]=0;
                terra_test[1]=0;
            }else{
                terra_test[0] = (terra_alt >> 8) & 0xFF;
                terra_test[1] = terra_alt & 0xFF;
            }
            sprintf(buff_terra,"T%s",terra_test);
            crc_terra = crc8_terra_SI((unsigned char*)buff_terra,3u);
            sm->data[0] = 'T';
            sm->data[1] = terra_test[0];
            sm->data[2] = terra_test[1];
            sm->data[3] = crc_terra;
            break;
		}
        if(sonar->encodeMode != SI_SONAR_TERRA){
            writePort( sonar->p, (char *)sm, sizeof( struct sonarMessage_ref ) );
        }else{
            writePort( sonar->p, (char *)sm, sizeof( struct sonarMessage_ref )-1 ); //send 4bytes instead of 5 for terraranger
        }

		// deal with counters
		sonar->lastUpdate += sonar->updateDt; /* sloppy, but probably more realistic */
	}

	/* PWM */

	if( ( st->time >= act->lastUpdatePWM + act->updateDtPWM ) &&
		( sim.mode == SIM_MODE_RUN ) ) {

		struct pwmServoMapping_ref    *map = act->pwmMap;
		struct pwmServoMixing_ref     *mix = act->pwmMix;
        struct pwmMTRServoMixing_ref  *pmtr = act->mtrMix;
		struct datalinkMessagePWM_ref *pwm = act->pwmFromSI;
		double delf[1], delt[1], delm[3], autopilot;

		delm[0] = gi->cntrlInput->roll->output; // roll
		delm[1] = gi->cntrlInput->pitch->output; // pitch
		delm[2] = gi->cntrlInput->rudder->output; // rudder
		delf[0] = gi->cntrlInput->throttle->output; // throttle
		delt[0] = gi->cntrlInput->throttle->output; // throttle

		if( co->autopilot == AUTOPILOT_OFF )
			autopilot = -1;
		else
			autopilot = 1;

		switch( map->mixMode ) {

		case MIX_3SWASH120DEG:
		case MIX_3SWASH90DEG:
			pwm->channel[mix->rightServo] = mix->biasR + (short)(delm[0]*mix->rollKR + delm[1]*mix->pitchKR + delf[0]*mix->collKR);
			pwm->channel[mix->leftServo]  = mix->biasL + (short)(delm[0]*mix->rollKL + delm[1]*mix->pitchKL + delf[0]*mix->collKL);
			pwm->channel[mix->backServo]  = mix->biasB + (short)(delm[0]*mix->rollKB + delm[1]*mix->pitchKB + delf[0]*mix->collKB);
			pwm->channel[mix->frontServo] = mix->biasF + (short)(delm[0]*mix->rollKF + delm[1]*mix->pitchKF + delf[0]*mix->collKF);
			pwm->channel[mix->yaw]        = mix->biasY + (short)(delm[2]*mix->gainY);
			pwm->channel[mix->throttle]   = mix->biasT + (short)(delt[0]*mix->gainT);
			pwm->channel[mix->autopilot]  = mix->biasA + (short)(autopilot*mix->gainA);

			pwm->channel[mix->rightServo] = LIMIT( pwm->channel[mix->rightServo], 1, 999 );
			pwm->channel[mix->leftServo]  = LIMIT( pwm->channel[mix->leftServo],  1, 999 );
			pwm->channel[mix->backServo]  = LIMIT( pwm->channel[mix->backServo],  1, 999 );
			pwm->channel[mix->yaw]        = LIMIT( pwm->channel[mix->yaw],        1, 999 );
			pwm->channel[mix->throttle]   = LIMIT( pwm->channel[mix->throttle],   1, 999 );
			break;

		default:
			pwm->channel[map->roll]      = LIMIT( map->bias[map->roll]      + (short)( delm[0]*  map->gain[map->roll]),      50, 950 );
			pwm->channel[map->pitch]     = LIMIT( map->bias[map->pitch]     + (short)( delm[1]*  map->gain[map->pitch]),     50, 950 );
			pwm->channel[map->yaw]       = LIMIT( map->bias[map->yaw]       + (short)( delm[2]*  map->gain[map->yaw]),       50, 950 );
			pwm->channel[map->thrust]    = LIMIT( map->bias[map->thrust]    + (short)( delf[0]*  map->gain[map->thrust]),    50, 950 );
			pwm->channel[map->throttle]  = LIMIT( map->bias[map->throttle]  + (short)( delt[0]*  map->gain[map->throttle]),  50, 950 );
			pwm->channel[map->autopilot] = LIMIT( map->bias[map->autopilot] + (short)( autopilot*map->gain[map->autopilot]), 50, 950 );
			break;

        case 99:
			break;
		} /* end switch on mixMode */

		/* send message */
		switch(act->sendPWM){
			default:
			 break;
			case 1: //normal way to send it
				pwm->messageID   = DATALINK_MESSAGE_PWM;
				pwm->messageSize = sizeof( struct datalinkMessagePWM_ref );
				datalinkCheckSumEncode( (unsigned char *)pwm, pwm->messageSize );
				writePort( act->pPWM, (char *)pwm, pwm->messageSize );
				break;
		}

		/* deal with counters */
		act->lastUpdatePWM += act->updateDtPWM; /* sloppy, but probably more realistic */

	}
	IFLIBEXT(ev_updateSi();)
		
		// send vehicle motion model as truth
		if ( s->vmm->enable && st->time > s->vmm->lastTime + s->vmm->dt && ( sim.mode == SIM_MODE_RUN ) )
		{
			struct datalinkMessageTruth_ref* out = s->vmm->truth;
			struct vehicle_ref* v = &vehicle;
			struct motionXforms_ref* xforms = v->motion->xforms;
			struct state_ref* state = v->motion->state;
			struct state_ref* std = v->motion->stateDot;	// to get accel

			unsigned int i;

			//out->time   = state->time;

			for ( i = 0; i < 3; i++ )
			{
				out->p_b_e_L[i] = state->p_b_e_L[i];
				out->v_b_e_L[i] = state->v_b_e_L[i];
				//out->w_b_e_L[i] = xforms->w_b_e_L[i];
				//out->v_b_e_B[i] = xforms->v_b_e_B[i];
				//out->w_b_e_B[i] = state->w_b_e_B[i];
				//out->a_b_e_L[i] = std->v_b_e_L[i];
			}
			//out->altitudeAGL = -out->p_b_e_L[2] - vehicleMotion.env->terrainAlt;

			// express acceleration in body axis
			//map_vector ( xforms->dcm_lb, out->a_b_e_L, out->a_b_e_B );

			for ( i = 0; i < 4; i++ )
			{
				out->q[i] = state->e[i];
			}

			//for ( i = 0; i < 3; i++ )
			//{
			//	for ( j = 0; j < 3; j++ )
			//	{
			//		out->dcm_lb[i][j] = xforms->dcm_lb[i][j];
			//	}
			//}
			//out->densityr = vehicleMotion.env->densityRatio;

			out->messageID = DATALINK_MESSAGE_TRUTH;
			out->messageSize = sizeof ( struct datalinkMessageTruth_ref );
			datalinkCheckSumEncode ( ( unsigned char* ) out, sizeof ( struct datalinkMessageTruth_ref ) );
			writePort ( s->vmm->p, ( char* ) out, sizeof ( struct datalinkMessageTruth_ref ) );

			s->vmm->lastTime = st->time;
		}
		else if ( sim.mode == SIM_MODE_INIT )
		{
			s->vmm->lastTime = -1;
		}
}

