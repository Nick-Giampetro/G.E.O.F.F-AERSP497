
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
 /* Copyright (c) Eric N. Johnson, 1998.  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <GL/glut.h>

#include "esim/db.h"
#include "esim/util.h"
#include "esim/quat.h"
#include "esim/sim_ref.h"
#include "esim/brwin_ref.h" /* for getDir thing */
#include "esim/command.h"
#include "esim/cnsl.h"
#include "esim/brwin.h"
#include "esim/db.h"
#include "esim/input_ref.h"
#include "rmax/motion_ref.h"
#include "rmax/onboard_ref.h"
#include "rmax/controller_ref.h"
#include "rmax/serial.h"
#include "rmax/datalink.h"
#include "rmax/gcs_ref.h"
#include "rmax/joyinputs_ref.h"
#include "rmax/gcs.h"
#include "rmax/joyinputs.h"
#include "rmax/panel.h"
#include "rmax/scene.h"
#include "rmax/logger.h"

#include "rmax/navigation_ref.h"
#include "rmax/sensors_ref.h"
#include "rmax/sensors.h"
#include "rmax/novatel.h"
#include "rmax/realScene_ref.h"
#include "rmax/scene_ref.h"
#include "rmax/generic_ref.h"
#include "rmax/matrix.h"
#include "rmax/checksum.h"

#define DRREAD_ERROR_NONE        0
#define DRREAD_ERROR_FILEOPEN    1
#define DRREAD_ERROR_PACKETORDER 2
#define DRREAD_ERROR_OLDPACKET   3
#define DRREAD_ERROR_UNKNOWN     4
#define DRREAD_ERROR_WRITE       5
#define IFLIBEXT(CODE)

void gcsAddVehicleName ( char* buffer, struct vehicleOutputs_ref* o )
{

	if ( o->uniqueID == 0 && o->model == 0 )
	{

		strcat ( buffer, "Aircraft" ); /* special case when nothing is set */

	}
	else
	{

		switch ( o->model )
		{
			case MODEL_MULTIROTOR:  strcat ( buffer, multirotorModel.name );      break;
			default:                sprintf ( buffer, "%sType[%d]", buffer, o->model );     break;
		}

		if ( o->uniqueID != 0 ) sprintf ( buffer, "%s%d", buffer, o->uniqueID ); /* 0 is "unknown" tail number */

	}

}


static void cleanFileName ( char* buffer )
{

	unsigned int i;

	/* replace characters that are not allowed */
	for ( i = 0; i < strlen ( buffer ); i++ )
	{
		switch ( buffer[i] )
		{
			default:
			break;
			case ' ':  /* allowed, but annoying to work with in a file name */
			case 92: /* \ */
			case 34: /* " */
			case 39: /* ' */
			case '/': case ':': case ';': case '*': case '?': case '<': case '>': case '|':
			case '%': case ',': case '#': case '$': case '!':
			case '+': case '{': case '}': case '&': case '[': case ']': case '•':
			case '-':  /* this is really allowed, but matlab doesn't like it */
			buffer[i] = '_';
			break;
		}
	}

}


static void gcsGetVehicleNameFileFriendly ( char* buffer, struct vehicleOutputs_ref* o )
{

	sprintf ( buffer, "" );
	gcsAddVehicleName ( buffer, o );
	cleanFileName ( buffer );

}


//static void gcsSetAReference( char *name ) {

	// Create a local copy of the passed in name to circumvent the immutuability
	// issues for potentially passed-in "aString" string constants.
//	char nameWithNumber[200];
//	char nameWithoutNumber[200];
//	char *number; // the '?' we want to change to the number of the active gcs

//	strcpy( nameWithNumber, name );

//	if( ( number = strchr( nameWithNumber, '?' ) ) != NULL ) {
//        *number = '\0';
//		sprintf( nameWithoutNumber, "%s%s", nameWithNumber, number + 1 );
//		*number = 48 + gcs.active; // ASCII 48 is '0'
//		setGlobalDirReference( nameWithoutNumber, nameWithNumber );
//	}
//
//}


static void recurseGcsSetReference ( Dir* dir )
{

	// Go through all variables in the directory to find sub-directories
	Var* var;
	int counter = 0;

	// Add to the references
	// TODO_JPD: FIXME: How do I do this without the question mark?
	// Assume the following pattern: "gcs<number>" is in the name

	char testName[200];
	char nameWithNumber[200];
	char nameWithoutNumber[200];
	char* number; // the character we want to change to the number of the active gcs

	strcpy ( nameWithNumber, dir->name );

	sprintf ( testName, "%s%d", "gcs", gcs.active ); // Create the test name "gcs<number>"
	if ( ( number = strstr ( nameWithNumber, testName ) ) != NULL )
	{
		number += 3; // Get after the "gcs"
		*number = '\0';
		sprintf ( nameWithoutNumber, "%s%s", nameWithNumber, number + 1 );
		*number = '0' + gcs.active;
		setGlobalDirReference ( nameWithoutNumber, nameWithNumber );
	}

	while ( counter < dir->size )
	{
		var = &dir->vars[counter];
		if ( var->type == TYPE_DIR )
		{
			/* This is a directory - recurse */
			recurseGcsSetReference ( var->child );
		}
		++counter;
	}

}


static void gcsSetReferences ( void )
{

	/* makes references to the active gcs structures */

	// Recurse through GCS instance directory to grab all directories and create
	//  local references
	char gcsInstanceName[200];
	//char *number = '\0';
	Dir* gcsDir;

	sprintf ( gcsInstanceName, "%s%d%s", "gcs", gcs.active, "Instance" ); // Create the instance name

	gcsDir = findDir ( gcsInstanceName );
	if ( gcsDir != NULL )
	{
		// Recurse through and set all references
		recurseGcsSetReference ( gcsDir );
	}

	//int i;
	//char buffer[100];

//NOTE: There is a (potential) issue with immutuability of passed in strings.
// When testing, all of the below properly worked with the previous version of
// gcsSetAReference, however, passing in "aString" segfaulted on linux as it
// (most likely) is interpreted as a string constant.
// As such, gcsSetAReference has been altered, see there for more details.
//
// Test cases that all worked:
//
//         char        test1[] = "1gcs?Set";
//   const char        test2[] = "2gcs?Set";
//         char const  test3[] = "3gcs?Set";
//   const char const  test4[] = "4gcs?Set";
//
//   gcsSetAReference( test1 );
//   gcsSetAReference( test2 );
//   gcsSetAReference( test3 );
//   gcsSetAReference( test4 );

/*	gcsSetAReference( "gcs?Set" );
	gcsSetAReference( "gcs?Outputs" );
	gcsSetAReference( "gcs?PortDatalink1" );
	gcsSetAReference( "gcs?PortDatalink2" );
	gcsSetAReference( "gcs?PortAnotherGCS" );
	gcsSetAReference( "gcs?dataport" );
	gcsSetAReference( "gcs?dataport2" );
	gcsSetAReference( "gcs?Datalink" );
	gcsSetAReference( "gcs?TrajMain" );
	gcsSetAReference( "panel?" );
	for( i=0; i<NUMPANELBOXES; i++ ) {
		sprintf( buffer, "panel?Box%d", i );
		gcsSetAReference( buffer );
	}
	gcsSetAReference( "limitCheck?" );
	gcsSetAReference( "gcs?TrajMove" );
	gcsSetAReference( "gcs?Timer" );
	gcsSetAReference( "camgrab?" );
	gcsSetAReference( "gcs?Ci" );
	gcsSetAReference( "gcs?AirLaunch" );

	gcsSetAReference( "gcs?DatalinkHeader" );
	gcsSetAReference( "gcs?DatalinkMessage0" );
	gcsSetAReference( "gcs?DatalinkMessage1" );
	gcsSetAReference( "gcs?DatalinkMessage1B" );
	gcsSetAReference( "gcs?DatalinkMessageUp0" );
	gcsSetAReference( "gcs?DatalinkMessageTrackingResults" );
	gcsSetAReference( "gcs?DatalinkMessageNavIPResults" );
	gcsSetAReference( "gcs?DatalinkMessagePTR" );
	gcsSetAReference( "gcs?DatalinkMessageDRRelay" );
	gcsSetAReference( "gcs?DatalinkMessageDRRelayReply" );
	gcsSetAReference( "gcs?DatalinkMessageVisionFormation" );
	gcsSetAReference( "gcs?dr_reconstruct" );
	gcsSetAReference( "gcs?DatalinkMessageSquitterMe" );
	gcsSetAReference( "gcs?DatalinkMessageSquitterOther" );
	gcsSetAReference( "gcs?DatalinkMessagePointPos" );
	gcsSetAReference( "gcs?DatalinkMessageSlip" );
	gcsSetAReference( "gcs?DatalinkMessageMapFP" );
	gcsSetAReference( "gcs?DatalinkMessageConfigFile" );
	gcsSetAReference( "gcs?DatalinkMessageConfigFileAck" );
	gcsSetAReference( "gcs?DatalinkMessageConfigFileDown" );
	gcsSetAReference( "gcs?DatalinkMessageConfigFileDownAck" );
	gcsSetAReference( "gcs?DatalinkMessageRCIF" );
	gcsSetAReference( "gcs?DatalinkMessageRCIFAck" );
	gcsSetAReference( "gcs?DatalinkMessageDatUpload" );
	gcsSetAReference( "gcs?DatalinkMessageDatUploadAck" );
	gcsSetAReference( "gcs?DatalinkMessageSlugStatus" );
	gcsSetAReference( "gcs?DatalinkMessageScanSimpleInfo" );
	gcsSetAReference( "gcs?DatalinkMessageScanSimple" );
	gcsSetAReference( "gcs?DatalinkMessageSlamData" );
	gcsSetAReference( "gcs?DatalinkMessage2dCov" );
	gcsSetAReference( "gcs?DatalinkMessageWorldPoints" );
*/
}


struct gcsInstance_ref* gcsGetInstance ( struct gcs_ref* g, int number )
{

	return g->instance0;

}


struct gcsInstance_ref* gcsActiveInstance ( struct gcs_ref* g )
{

	return g->instance0;

}

static unsigned char checkSystemSafety ( void )
{

	struct gcsInstance_ref* gi = gcsActiveInstance ( &gcs );
	struct gcsDatalink_ref* data = gi->datalink;

	// Set the value of the safety flag
	if ( ( gi->traj->lockMotorStart ) && ( data->m0->wow ) && ( data->m0->motor == 0 ) )
	{
		// The lock is set, the vehicle is on the ground, and the motor is not running
		//  Do not perform the action
		return 0;
	}

	// The caller can perform the action
	return 1;
}

void gcsAddLine ( struct gcsInstance_ref* gi, const char* text )
{

	char textBuffer[400];
	time_t current_time;
	struct tm* local_time;

	current_time = time ( NULL );
	local_time = localtime ( &current_time );

	sprintf ( textBuffer, "> %s  (%02d:%02d:%02d)",
						text,
						/*(1900 + local_time->tm_year)%100,
						local_time->tm_mon + 1,
						local_time->tm_mday,*/
						local_time->tm_hour,
						local_time->tm_min,
						local_time->tm_sec );
	if ( gi != NULL )
	{
		sprintf ( textBuffer, "%s%.2f", textBuffer, gi->datalink->m0->time );
	}

	logInfo ( textBuffer );

}

static void dumpMessage ( struct gcsInstance_ref* gi, int messageID )
{

	if ( gi->datalink->makeCSV )
	{

		struct dir_ref* data_dir = &gcs0DatalinkMessage0_dir;
		char buffer[256];

		int i;
		void* fd;

		switch ( messageID )
		{
			case DATALINK_MESSAGE0:
			if ( gi == &gcs0Instance ) data_dir = &gcs0DatalinkMessage0_dir;
			fd = gi->datalink->makeCSVFD_0;
			break;
			case DATALINK_MESSAGE1:
			if ( gi == &gcs0Instance ) data_dir = &gcs0DatalinkMessage1_dir;
			fd = gi->datalink->makeCSVFD_1;
			break;
			default:
			return; /* not a supported directory */
			break;
		}

		if ( fd != 0 )
		{
			for ( i = 0; i < data_dir->size; i++ )
			{
				brwinFormatValue ( buffer, &( data_dir->vars[i] ) );
				fprintf ( ( FILE* ) fd, buffer );
				if ( i < data_dir->size - 1 ) fprintf ( ( FILE* ) fd, "," );
				else                       fprintf ( ( FILE* ) fd, "\n" );
			}
		}

	}

}



static unsigned char doButtonBlend ( unsigned char first, unsigned char second )
{

	return MAX ( first, second );

}


static void doUp0buttons ( struct gcsInstance_ref* gi )
{

	struct gcsDatalink_ref* data = gi->datalink;

	data->up0->button[4] = ( char ) ( gi->cntrlInput->air->output );
	data->up0->button[6] = ( char ) ( gi->cntrlInput->suppressStick->output || gi->cntrlInput->ground->output );
	data->up0->button[3] = ( char ) ( gi->cntrlInput->ground->output || gi->cntrlInput->safeOn->output );
	data->up0->button[2] = ( char ) ( gi->cntrlInput->safeOff->output );
	data->up0->button[0] = ( char ) ( gi->cntrlInput->arm->output );
	data->up0->button[1] = ( char ) ( gi->cntrlInput->manOverride->output );
	// The next two are an OR condition since digitalFunction[11] means both buttons are pressed
	data->up0->button[12] = ( char ) ( gi->cntrlInput->gpsDenied->output || gi->cntrlInput->gpsDenRestGps->output );
	data->up0->button[11] = ( char ) ( gi->cntrlInput->gpsDenied->output || gi->cntrlInput->gpsDenRestPos->output );
	data->up0->button[13] = ( char ) ( gi->cntrlInput->systemSafety->output );

	// JPD: Old mapping
	//data->up0->button[4]     = motionControls.wowUpButton;
	//data->up0->button[6]     = motionControls.wowDownButton;
	//data->up0->button[0]     = motionControls.safeOnButton;
	//data->up0->button[2]     = motionControls.safeOffButton;
	//data->up0->button[10]    = motionControls.goFastButton;
	//data->up0->button[15]    = motionControls.manualOverride;
	//data->up0->button[12]    = motionControls.gpsDeniedButton1;
	//data->up0->button[11]    = motionControls.gpsDeniedButton2;

	/* hat switch - currently no functions are defined */

}

static void readDatalink ( struct gcs_ref* g, struct gcsInstance_ref* gi, struct serialPort_ref* port,
													 char* status, unsigned char theActiveInstance )
{

	struct gcsDatalink_ref* data = gi->datalink;
	struct onboard_ref* ob = &onboard; /* for flightplan */
	struct sceneGlobal_ref* sg = &sceneGlobal; /* for storing laser scan data on incoming message */

	unsigned char* bf;
	char buffer[RMTCNSL_MAXSIZE + 50]; // This has to be bigger than the datalink message length to account for added information on this end
	int j, done, index;
	void* dataPtr;
	int size;
	double readUntilTime = -1.0;
	int doOnce = 1;
	long lastReceived = -1;
	double timeBias;
	int i = 0;
	int receivedLOGODD = 0;
	int receivedActivityFlag = 0;

#ifdef PSP
	double viconq[4];
#endif

	if ( port->dataSource == PORT_PLAYBACK )
	{
		if ( port->init )
		{ /* got reset */
			data->lastPlaybackWallTime = sim.wallTime;
		}
		if ( data->lastPlaybackRate != data->playbackRate )
		{ /* in case playback rate is changed */
			data->lastPlaybackWallTime = sim.wallTime;
			data->lastPlaybackRate = data->playbackRate;
		}

		readUntilTime = data->lastPlaybackTime + ( sim.wallTime - data->lastPlaybackWallTime ) * data->playbackRate;
	}
	else
	{
		readUntilTime = -1.0;
	}

	lastReceived = -1;
	doOnce = 1;
	while ( ( data->m0->time < readUntilTime && lastReceived < ( signed long ) port->received ) ||
					( port->dataSource != PORT_PLAYBACK && doOnce ) )
	{

		lastReceived = port->received;

		readPort ( port );

		if ( port->dataSource == PORT_OFF ) return;

		doOnce = 0;

		done = 0;
		index = 0;

		while ( ( index <= port->bytesread - ( int ) sizeof ( struct datalinkHeader_ref ) ) && !done )
		{

			if ( ( port->buffer[index] == DATALINK_SYNC0 ) &&
					 ( port->buffer[index + 1] == DATALINK_SYNC1 ) &&
					 ( port->buffer[index + 2] == DATALINK_SYNC2 ) )
			{

				bf = &( port->buffer[index] );

				memcpy ( data->header, bf, sizeof ( struct datalinkHeader_ref ) );

				if ( datalinkCheckSumCompute ( bf, sizeof ( struct datalinkHeader_ref ) - sizeof ( int ) * 2 ) == data->header->hcsum &&
						 data->header->messageSize >= sizeof ( struct datalinkHeader_ref ) &&
						 data->header->messageSize < BUFFERSIZE )
				{

					if ( data->header->messageSize + index <= port->bytesread )
					{
						/* have read in the entire message */

						/*printf( "got here!\n" );*/

						/*((struct datalinkHeader_ref *)bf)->hcsum = 0;*/
						if ( datalinkCheckSumCompute ( &bf[sizeof ( struct datalinkHeader_ref )], data->header->messageSize - sizeof ( struct datalinkHeader_ref ) ) == data->header->csum )
						{

							*status = DATALINK_ON;

							switch ( data->header->messageID )
							{
								case DATALINK_MESSAGE0:
								if ( data->header->messageSize == sizeof ( struct datalinkMessage0_ref ) )
								{
									memcpy ( data->m0, bf, sizeof ( struct datalinkMessage0_ref ) );
									dumpMessage ( gi, DATALINK_MESSAGE0 );

									if ( port->dataSource == PORT_PLAYBACK )
									{
										if ( data->m0->time <= data->lastPlaybackTime || data->m0->time > data->lastPlaybackTime + 5.0 )
										{
											data->lastPlaybackWallTime = sim.wallTime;
											readUntilTime = 0; /* we got reset, kick out */
										}
										else
										{
											data->lastPlaybackWallTime += MAX ( ( data->m0->time - data->lastPlaybackTime ) / MAX ( data->playbackRate, 0.01 ), 0 );
										}
										data->lastPlaybackTime = data->m0->time;
									}

									if ( data->m0->time < data->time - 15 )
									{ /* the 15 is to make sure this is really a restart and not just a late message */
										for ( j = 0; j < PLANEDIT_NPLANS; j++ )
											gi->traj->uploadEach[j] = 1; /* onboard may have been restarted */
										/* blow away downloaded stuff */
										gi->traj->flightPlanDL->lastIndex = 0;
										gi->traj->flightPlanLCDL->lastIndex = 0;
										data->time = data->m0->time;
									}

									if ( data->m0->time >= data->time )
									{ /* throw out messages if out of order */

/* timer functionality */
										if ( gi->timer->wow != MIN ( data->m0->wow, 1 ) )
										{
											if ( data->m0->wow == 0 )
											{
												gi->timer->startTime = data->m0->time;
											}
											else
											{
												gi->timer->prevTime += gi->timer->time;
											}
											gi->timer->wow = MIN ( data->m0->wow, 1 );
										}
										if ( gi->timer->wow == 0 )
										{
											if ( gi->timer->startTime > data->m0->time ) gi->timer->startTime = data->m0->time;
											gi->timer->time = data->m0->time - gi->timer->startTime;
										}
										else
										{
											gi->timer->time = 0;
										}
										gi->timer->flightTime = gi->timer->time + gi->timer->prevTime;
										/* end timer function */

										/* managing time */
										data->time = data->m0->time;
										timeBias = data->time - sim.time;
										if ( ABS ( timeBias - data->timeBias ) > data->maxExtrap ) data->timeBias = timeBias;
										data->timeBias += ( timeBias - data->timeBias ) * data->timeBiasFilterGain;

										for ( j = 0; j < 4; j++ )
											data->q[j] = data->m0->q[j];
										build_rotmatrix ( data->dcm_bl, data->q );
										matrix_transpose ( data->dcm_bl, data->dcm_lb );

										data->navStatus = data->m0->navStatus;
										data->gpsStatus = data->m0->gpsStatus;
										data->aglStatus = data->m0->aglStatus;
										data->autopilot = data->m0->autopilot;
										if ( !data->overrun )
											data->overrun = data->m0->overrun;

										data->terrainH = -data->m0->altitudeAGL - data->m0->pos[2];
									}
									else
									{
										int test = 0;
									}
								}
								break;

								case DATALINK_MESSAGE1:
								if ( data->header->messageSize == sizeof ( struct datalinkMessage1_ref ) )
								{
									double timeSkew;
									unsigned char camera;
									memcpy ( data->m1, bf, sizeof ( struct datalinkMessage1_ref ) );
									dumpMessage ( gi, DATALINK_MESSAGE1 );

									gi->outputs->model = data->m1->type;
									gi->outputs->uniqueID = data->m1->uniqueID;

									timeSkew = sim.time + data->timeBias - data->m1->time;

									/* energy use estimate */
									if ( data->m1->time > gi->timer->mAh_time )
									{
										gi->timer->mAh += MIN ( data->m1->time - gi->timer->mAh_time, gi->timer->mAh_dtMax ) * ( double ) ( data->m1->current ) / 3600;
									}
									gi->timer->mAh_time = data->m1->time;

									for ( j = 0; j < 3; j++ )
									{
										/*data->field_B[j] = data->m1->field_B[j];*/
										gi->traj->traj->a[j] = data->m1->traj_a[j];
										gi->traj->traj->v[j] = data->m1->traj_v[j] + timeSkew * data->m1->traj_a[j];
										gi->traj->traj->x[j] = data->m1->traj_x[j] + timeSkew * data->m1->traj_v[j];
										gi->traj->traj->q[j] = data->m1->traj_q[j];
										if ( theActiveInstance && onboard.run == 0 )
										{
											ob->actuators->work->delm[j] = data->m1->delm[j]; /* copying these over for some code in planner.cpp */
											ob->navigation->out->v_a_e_L[j] = data->m1->wind[j];
										}
									}
									gi->traj->traj->q[3] = data->m1->traj_q[3];
									gi->traj->traj->psi = data->m1->traj_psi;
									if ( theActiveInstance && onboard.run == 0 )
									{
										ob->actuators->work->delf[0] = data->m1->delf[0]; /* copying these over for some code in planner.cpp */
										ob->actuators->work->delt[0] = data->m1->delt[0];
									}
									gi->traj->traj->stationKeep = 0;
									gi->traj->traj->formationLanding = 0;
									switch ( data->m1->safemode )
									{
										case 0:  gi->traj->traj->safemode = 0;  break;
										case 1:  gi->traj->traj->safemode = 1;  break;
										case 2:  gi->traj->traj->safemode = 2;  break;
										case 3:  gi->traj->traj->safemode = 0;  break;
										case 5:  gi->traj->traj->safemode = 1;  break;
										case 6:  gi->traj->traj->safemode = 0;  break;
										case 7:  gi->traj->traj->safemode = 0;  break;
										case 8:  gi->traj->traj->safemode = 0;  break;
										case 11: gi->traj->traj->safemode = 0;  break;
										case 12: gi->traj->traj->safemode = 0;  gi->traj->traj->stationKeep = 1;  break; /* station keeping */
										case 13: gi->traj->traj->safemode = 0;  gi->traj->traj->formationLanding = 1; break; /* formation landing */
										default:                                break; /* leave it alone for other cases */
									}
									if ( gi->traj->traj->geofence )
									{ /* ground planner detecting an issue - display if nothing else going on */
										if ( data->m1->safemode == 0 ) data->m1->safemode = 10;
										if ( data->m1->safemode == 1 ) data->m1->safemode = 10;
										gi->traj->traj->geofence = MAX ( gi->traj->traj->geofence - 1, 0 );
									}

									gi->traj->traj->manIndex = LIMIT ( data->m1->traj_manIndex, 0, MAN_NMANS );
									gi->traj->traj->status = data->m1->traj_status;
									gi->traj->nav->vscale = data->m1->traj_vscale;

									camera = data->m1->cameraControlStatus;
									data->m1->cameraControlStatus &= 0xf;
									camera -= data->m1->cameraControlStatus;
									camera = camera >> 4;
									switch ( camera )
									{
										case 0:
										default:
										gi->outputs->pan = data->m1->pan;
										gi->outputs->tilt = data->m1->tilt;
										gi->outputs->roll = data->m1->roll;
										gi->outputs->fovy = data->m1->fovy;
										break;
										case 1:
										gi->outputs->pan2 = data->m1->pan;
										gi->outputs->tilt2 = data->m1->tilt;
										gi->outputs->roll2 = data->m1->roll;
										gi->outputs->fovy2 = data->m1->fovy;
										break;
										case 2:
										gi->outputs->pan3 = data->m1->pan;
										gi->outputs->tilt3 = data->m1->tilt;
										gi->outputs->roll3 = data->m1->roll;
										gi->outputs->fovy3 = data->m1->fovy;
										break;
										case 3:
										gi->outputs->pan4 = data->m1->pan;
										gi->outputs->tilt4 = data->m1->tilt;
										gi->outputs->roll4 = data->m1->roll;
										gi->outputs->fovy4 = data->m1->fovy;
										break;
									}

									data->magnetStatus = data->m1->magnetStatus;
									gi->traj->nav->pointPos[0] = data->m1->pointPos[0];
									gi->traj->nav->pointPos[1] = data->m1->pointPos[1];
									gi->traj->nav->pointPos[2] = data->m1->pointPos[2];
									/*navinit.datumLat = data->m1->datumLat;
									navinit.datumLon = data->m1->datumLon;
									navinit.datumAlt = data->m1->datumAlt;*/
									if ( data->m1->lostComm )
									{
										for ( j = 0; j < PLANEDIT_LC; j++ )
											gi->traj->uploadEach[j] = 1; /* lost comm was triggered, waypoints replaced */
									}
								}
								break;

								case DATALINK_MESSAGE_SENDDIR:
								if ( data->header->messageSize <= SENDDIR_MAXSIZE )
								{
									memcpy ( data->senddir, bf, data->header->messageSize );
									dataPtr = getDirDataPointer ( data->senddir->dirNum, &size );
									if ( data->senddir->uniqueID != data->senddirID )
									{
										data->senddirID = data->senddir->uniqueID;
										if ( size == data->senddir->size )
										{
											memcpy ( dataPtr, data->senddir->data, size );
											sprintf ( buffer, "directory (%d) received from ", data->senddir->dirNum );
										}
										else
										{
											sprintf ( buffer, "problem with dir (%d) from ", data->senddir->dirNum );
										}
										gcsAddVehicleName ( buffer, gi->outputs );
										if ( data->senddir->verbose ) gcsAddLine ( gi, buffer );
									}
								}
								break;

								case DATALINK_MESSAGE_RMTCNSL:
								/* the <= thing isn't as stringent as it could be, but this allows backward compatibility */
								/* with older fixed-size version of this packet */
								if ( data->header->messageSize <= sizeof ( struct datalinkMessageRmtCnsl_ref ) )
								{
									memcpy ( data->rmtCnsl, bf, data->header->messageSize );
									if ( data->rmtCnsl->uniqueID != data->lastRmtCnsl )
									{
										/* need a more robust way to do this to handle multiple lines being sent rapidly
											over two links */
										if ( strlen ( ( char* ) data->rmtCnsl->data ) < RMTCNSL_MAXSIZE )
										{
											sprintf ( buffer, "%s /", ( char* ) data->rmtCnsl->data );
											gcsAddVehicleName ( buffer, gi->outputs );
											gcsAddLine ( gi, buffer );
											if ( sceneGlobal.showRmtCnsl ) sceneMessageAll ( buffer );

											if ( gi->set->showLogOverrunMsg && data->rmtCnsl->nOverruns > 1 )
											{
												sprintf ( buffer, "%d log overruns occured since last log message  /", data->rmtCnsl->nOverruns );
												gcsAddVehicleName ( buffer, gi->outputs );
												gcsAddLine ( gi, buffer );
												if ( sceneGlobal.showRmtCnsl ) sceneMessageAll ( buffer );
											}
										}
										data->lastRmtCnsl = data->rmtCnsl->uniqueID;
									}  /* ignore duplicates */
								}
								break;

								case DATALINK_MESSAGE_AUTOPILOTDELS:
								if ( data->header->messageSize == sizeof ( struct datalinkMessageAutopilotDels_ref ) )
								{
									memcpy ( data->autopilotDels, bf, sizeof ( struct datalinkMessageAutopilotDels_ref ) );
								}
								break;

								default:
								/* unrecognized type */
								break;
							}

							data->itime++;
						}
						else
						{
							sprintf ( buffer, "bad message checksum from " );
							gcsAddVehicleName ( buffer, gi->outputs );
							gcsAddLine ( gi, buffer );
						}
						index += data->header->messageSize - 1;

					}
					else
					{ /* have not read all of message yet */
						 /*printf( "message not read yet\n" );*/
						index--;
						done = 1;
					}
				}
				else
				{ /* header checksum is bad */
					index += sizeof ( struct datalinkHeader_ref ) - 1;
					//i = BUFFERSIZE;
					sprintf ( buffer, "bad header checksum from " );
					gcsAddVehicleName ( buffer, gi->outputs );
					gcsAddLine ( gi, buffer );
					//port->init = 1;
				}
			}
			index++;

			if ( index < 0 ) index = BUFFERSIZE - 1;
		}
		clearPort ( port, index );  /* remove first i bytes from buffer */

	}

}

void simStartLocationCmd ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	char buffer[200];

	switch ( argc )
	{
		default:
		logInfo ( "usage: simStartLocation latitude(deg) longitude(deg) altitude(ft)" );
		logInfo ( "       simStartLocation clicked" );
		break;
		case 2:
		if ( !strcmp ( argv[1], "clicked" ) )
		{
			navinit.datumLat = gi->outputs->datumLat + C_FT2NM / 60 * gi->traj->nav->pointPos[0];
			navinit.datumLon = hmodDeg ( gi->outputs->datumLon + C_FT2NM / 60 * gi->traj->nav->pointPos[1] / cos ( gi->outputs->datumLat * C_DEG2RAD ) );
			gi->traj->nav->pointPos[0] = 0;
			gi->traj->nav->pointPos[1] = 0;
			sprintf ( buffer, "simStartLocation: start location set for " );
			gcsAddVehicleName ( buffer, gi->outputs );
			logInfo ( buffer );
		}
		else
		{
			logInfo ( "usage: simStartLocation clicked" );
		}
		break;
		case 4:
		navinit.datumLat = atof ( argv[1] );
		navinit.datumLon = atof ( argv[2] );
		navinit.datumAlt = atof ( argv[3] );
		sprintf ( buffer, "simStartLocation: start location set for " );
		gcsAddVehicleName ( buffer, gi->outputs );
		logInfo ( buffer );
		break;
	}

}

void gpscalcCmd ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	char buffer[400];
	double latitude, longitude;
	double x, y;

	if ( argc == 3 )
	{
		latitude = atof ( argv[1] );
		longitude = atof ( argv[2] );

		x = ( latitude - gi->outputs->datumLat ) * C_NM2FT * 60;
		y = hmodDeg ( longitude - gi->outputs->datumLon ) * C_NM2FT * 60
			* cos ( gi->outputs->datumLat * C_DEG2RAD );

		sprintf ( buffer, "(x,y)=(%.0f,%.0f)ft", x, y );

	}
	else
		sprintf ( buffer, "usage: gpscalc lat long (decimal degrees)" );
	gcsAddLine ( gi, buffer );

}

void gpsrefConfigure ( int argc, char** argv )
{

	struct gpsRef_ref* gps = &gpsRef;

	gps->configState = 1;
	gps->calibrate = 0;
	gps->mobile = 0;

#if 0 /* old code */
	struct gcs_ref* g = &gcs;
	struct gpsRef_ref* ref = g->gpsRef;

	//char buffer[200];
	ref->configure = 1;
	ref->calibrate = 0; /* done callibrating */
	ref->mobile = 0;

	writePortText ( &gcsPortGPSRef, "unlogall\r" );

	if ( ref->useNovatelMsg == 0 )
	{
		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "interfacemode com1 novatel rtca\r" );
	}
	else
	{
		// don't really need this, but just in case
		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "interfacemode com1 novatel novatel\r" );
	}

	serialSleep ( 500 );
	sprintf ( buffer, "fix position %.12lf %.12lf %.3f\r",
						ref->gpsrefLat, ref->gpsrefLon, ref->gpsrefAlt );
	writePortText ( &gcsPortGPSRef, buffer );

	if ( ref->useNovatelMsg == 0 )
	{
		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtcaref,ontime,10\r" );

		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtcaobs,ontime,1\r" ); /* fast = 1  slow = 2 */

		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtca1,ontime,10,1\r" ); /* fast = 5,1  slow = 10, 3 */

		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtcaephem,ontime,10,7\r" );
	}
	else
	{
		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtcarefb,ontime,10\r" );

		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtcaobsb,ontime,1\r" ); /* fast = 1  slow = 2 */

		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtca1b,ontime,10,1\r" ); /* fast = 5,1  slow = 10, 3 */

		serialSleep ( 500 );
		writePortText ( &gcsPortGPSRef, "log com1,rtcaephemb,ontime,10,7\r" );
	}
	// now do the setup for com2
	//serialSleep( 500 );
	//writePortText( &gcsPortGPSRefRTCM, "interfacemode com2 novatel rtcm\r" );

	//serialSleep( 500 );
	//writePortText( &gcsPortGPSRefRTCM, "log com2 rtcm1 ontime 1\r" );

	commandExecute ( "gpsrefSave temp_gpsref" );
	gcsAddLine ( "gps reference configured" );
#endif

}

void gpsrefCalibrate ( int argc, char** argv )
{

	struct gpsRef_ref* gps = &gpsRef;

	gps->samples = 0;
	gps->calibrate = 1;
	gps->configState = 1;
	gps->mobile = 0;
	gps->addedChecklistItem = 0;

#if 0 /* old code */
	struct gcs_ref* g = &gcs;
	struct gpsRef_ref* ref = g->gpsRef;

	writePortText ( &gcsPortGPSRef, "unlogall\r" );

	serialSleep ( 500 );
	writePortText ( &gcsPortGPSRef, "interfacemode com1 novatel novatel\r" );

	serialSleep ( 500 );
	writePortText ( &gcsPortGPSRef, "fix none\r" );

	serialSleep ( 500 );
	writePortText ( &gcsPortGPSRef, "log com1 bestposb ontime 1\r" );

	ref->samples = 0;
	ref->calibrate = 1;
	ref->mobile = 0;
#endif
}

void gpsrefMobile ( int argc, char** argv )
{

	struct gpsRef_ref* gps = &gpsRef;

	gps->calibrate = 0;
	gps->mobile = 1;
	gps->configState = 1;

#if 0 /* old code*/
	struct gcs_ref* g = &gcs;
	struct gpsRef_ref* ref = g->gpsRef;
	char buffer[80];

	writePortText ( &gcsPortGPSRef, "unlogall\r" );

	serialSleep ( 500 );
	writePortText ( &gcsPortGPSRef, "interfacemode com1 novatel novatel\r" );

	serialSleep ( 500 );
	writePortText ( &gcsPortGPSRef, "fix none\r" );

	serialSleep ( 500 );
	sprintf ( buffer, "log com1 bestposb ontime %f\r", ref->updateDtMobile );
	writePortText ( &gcsPortGPSRef, buffer );

	serialSleep ( 500 );
	sprintf ( buffer, "log com1 bestvelb ontime %f\r", ref->updateDtMobile );
	writePortText ( &gcsPortGPSRef, buffer );

	ref->calibrate = 0;
	ref->mobile = 1;
#endif
}

void gpsrefAlign ( int argc, char** argv )
{

	struct gpsRef_ref* gps = &gpsRef;

	gps->calibrate = 0;
	gps->mobile = 2;
	gps->configState = 1;
}

void gpsrefDatum ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsSet_ref* set = gi->set;
	struct vehicleOutputs_ref* out = gi->outputs;

	set->datumLat = out->latitude;
	set->datumLon = out->longitude;
	set->datumAlt = out->altitudeMSL - out->zgear;

	gcsAddLine ( gi, "datum set, ready for navInit" );

}

void tellGpsref ( int argc, char** argv )
{

	char buffer[400];
	char newl[100];
	int i;

	if ( argc > 1 )
	{
		sprintf ( buffer, "%s", argv[1] );
		if ( argc > 2 )
			for ( i = 2; i < argc; i++ )
			{
				sprintf ( newl, " %s", argv[i] );
				strcat ( buffer, newl );
			}
		sprintf ( newl, "\r" );
		strcat ( buffer, newl );
		writePortText ( &gcsPortGPSRef, buffer );
		sprintf ( newl, " sent to gpsref" );
		strcat ( buffer, newl );
	}
	else
		sprintf ( buffer, "usage: gpsref <test to send>" );
	gcsAddLine ( NULL, buffer );

}

void pointLatLong ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsTrajectoryMain_ref* gt = gi->traj;
	char   buffer[400];
	double latitude;
	double longitude;
	double x;
	double y;

	if ( argc == 3 )
	{

		latitude = atof ( argv[1] );
		longitude = atof ( argv[2] );

		x = ( latitude - gi->outputs->datumLat ) * C_NM2FT * 60;
		y = hmodDeg ( longitude - gi->outputs->datumLon ) * C_NM2FT * 60
			* cos ( gi->outputs->datumLat * C_DEG2RAD );

		//gcsSendPointPos( x, y, gi->traj->nav->pointPos[2] );

		sprintf ( buffer, "point to (%.5f,%.5f)", latitude, longitude );

	}
	else
		sprintf ( buffer, "usage: pointLatLong lat long (decimal degrees)" );
	gcsAddLine ( gi, buffer );

}


void remoteCommand ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;

	char buffer[400];
	char newl[100];
	int i;

	if ( argc > 1 )
	{
		sprintf ( buffer, "%s", argv[1] );
		if ( argc > 2 )
			for ( i = 2; i < argc; i++ )
			{
				sprintf ( newl, " %s", argv[i] );
				strcat ( buffer, newl );
			}
		/*sprintf( newl, "\r" );
		strcat( buffer, newl );*/

		/* encode message */

		data->rmtCmd->size = MIN ( strlen ( buffer ) + 1, RMTCMD_MAXSIZE ); /* plus 1 to get the null */
		memcpy ( data->rmtCmd->data, buffer, data->rmtCmd->size );
		data->rmtCmd->uniqueID++; //FIXME: This is not necessarily safe if multiple GCS instances are operating.

		data->rmtCmd->messageID = DATALINK_MESSAGE_RMTCMD;
		if ( data->useOldRmtCmd == 1 )
		{
			data->rmtCmd->messageSize = sizeof ( struct datalinkMessageRmtCmd_ref );
		}
		else
		{
			data->rmtCmd->messageSize = sizeof ( struct datalinkMessageRmtCmd_ref ) - RMTCMD_MAXSIZE + data->rmtCmd->size;
		}

		/* send message */
		datalinkCheckSumEncode ( ( unsigned char* ) data->rmtCmd, data->rmtCmd->messageSize );
		writePort ( data->p1, ( char* ) data->rmtCmd, data->rmtCmd->messageSize );
		writePort ( data->p2, ( char* ) data->rmtCmd, data->rmtCmd->messageSize );
		data->lastUplink = sim.time;

		sprintf ( newl, " command sent to " );
		strcat ( buffer, newl );
		gcsAddVehicleName ( buffer, gi->outputs );
	}
	else
		sprintf ( buffer, "usage: rc <text to send>" );

	gcsAddLine ( gi, buffer );

}


void remoteCommandToInstance ( struct gcsInstance_ref* gi, char* buffer )
{
	struct gcsDatalink_ref* data = gi->datalink;
	char newl[100];

	/* encode message */

	data->rmtCmd->size = MIN ( strlen ( buffer ) + 1, RMTCMD_MAXSIZE ); /* plus 1 to get the null */
	memcpy ( data->rmtCmd->data, buffer, data->rmtCmd->size );
	data->rmtCmd->uniqueID++; //FIXME: This is not necessarily safe if multiple GCS instances are operating.

	data->rmtCmd->messageID = DATALINK_MESSAGE_RMTCMD;
	if ( data->useOldRmtCmd == 1 )
	{
		data->rmtCmd->messageSize = sizeof ( struct datalinkMessageRmtCmd_ref );
	}
	else
	{
		data->rmtCmd->messageSize = sizeof ( struct datalinkMessageRmtCmd_ref ) - RMTCMD_MAXSIZE + data->rmtCmd->size;
	}

	/* send message */
	datalinkCheckSumEncode ( ( unsigned char* ) data->rmtCmd, data->rmtCmd->messageSize );
	writePort ( data->p1, ( char* ) data->rmtCmd, data->rmtCmd->messageSize );
	writePort ( data->p2, ( char* ) data->rmtCmd, data->rmtCmd->messageSize );
	data->lastUplink = sim.time;

	sprintf ( newl, " command sent to " );
	strcat ( buffer, newl );
	gcsAddVehicleName ( buffer, gi->outputs );

	gcsAddLine ( gi, buffer );
}

void resetTimer ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	char buffer[200];

	gi->timer->prevTime = 0;

	sprintf ( buffer, "flight timer reset for " );
	gcsAddVehicleName ( buffer, gi->outputs );
	gcsAddLine ( gi, buffer );

}

static void gcsOutputUpdate ( struct gcs_ref* g, struct gcsInstance_ref* gi, unsigned char theActiveInstance )
{

	struct gcsSet_ref* set = gi->set;
	struct gcsDatalink_ref* data = gi->datalink;
	struct vehicleOutputs_ref* o = gi->outputs;
	struct flightPlan_ref* fp;

	double dcm_wb[3][3], dcm90[3][3];
	double phi, theta, psi;
	double sal, cal, sbe, cbe;
	double v_b_e_L[3], v_b_a_L[3], v_b_a_B[3];
	double xa, dt;
	int i;

	/* position */

	dt = LIMIT ( sim.time + data->timeBias - data->m0->time, -data->maxExtrap, data->maxExtrap );
	o->time = data->m0->time + dt;
	for ( i = 0; i < 3; i++ )
	{
		o->pos[i] = data->m0->pos[i] + data->m0->vel[i] * dt;
	}
	xa = data->m0->altitudeAGL - data->m0->vel[2] * dt;

	gi->traj->nav->time = o->time;
	for ( i = 0; i < 3; i++ )
	{
		gi->traj->nav->p_b_e_L[i] = o->pos[i];
		gi->traj->nav->v_b_e_L[i] = data->m0->vel[i];
		gi->traj->nav->v_a_e_L[i] = data->m1->wind[i];
	}
	for ( i = 0; i < 4; i++ )
	{
		gi->traj->nav->q[i] = data->m0->q[i];
	}
	gi->traj->nav->terrainH = data->terrainH;
	gi->traj->nav->densityr = exp ( -o->altitudeMSL / 36000 );
	gi->traj->nav->altitudeAGL = data->m0->altitudeAGL;

	o->datumLat = data->m1->datumLat;
	o->datumLon = data->m1->datumLon;
	o->datumAlt = data->m1->datumAlt;

	o->zgear = set->zgear;
	o->latitude = ( o->datumLat + o->pos[0] / ( 60 * C_NM2FT ) );
	o->longitude = hmodDeg ( ( o->datumLon + o->pos[1] / ( 60 * C_NM2FT )
														 / cos ( o->datumLat * C_DEG2RAD ) ) );
	o->altitudeMSL = -o->pos[2] + o->datumAlt;
	o->altitudeAGL = xa - set->zgear;
	o->terrainAlt = o->altitudeMSL - o->zgear - o->altitudeAGL;

	/* attitude */

	saveFloat ( data->dcm_bl, o->float_dcm_bl );
	saveFloat ( data->dcm_lb, o->float_dcm_lb );
	dcm2euler ( data->dcm_bl, &phi, &theta, &psi );
	o->phi = phi * C_RAD2DEG;
	o->theta = theta * C_RAD2DEG;
	o->psi = psi * C_RAD2DEG;
	dcm90[0][0] = -data->dcm_bl[0][2];
	dcm90[0][1] = data->dcm_bl[0][1];
	dcm90[0][2] = data->dcm_bl[0][0];
	dcm90[1][0] = -data->dcm_bl[1][2];
	dcm90[1][1] = data->dcm_bl[1][1];
	dcm90[1][2] = data->dcm_bl[1][0];
	dcm90[2][0] = -data->dcm_bl[2][2];
	dcm90[2][1] = data->dcm_bl[2][1];
	dcm90[2][2] = data->dcm_bl[2][0];
	dcm2euler ( dcm90, &o->phi90, &o->theta90, &o->psi90 );
	o->phi90 *= C_RAD2DEG;
	o->theta90 *= C_RAD2DEG;
	o->psi90 *= C_RAD2DEG;

	if ( vehicle.run )
	{
		o->thetaFuse = vehicleOutputs.thetaFuse; // this is freewing fuselage angle
	}

	/* velocity */

	for ( i = 0; i < 3; i++ )
		v_b_e_L[i] = data->m0->vel[i];
	map_vector ( data->dcm_lb, v_b_e_L, o->velocity );

	/* assume sea level density */

	for ( i = 0; i < 3; i++ )
	{
		v_b_a_L[i] = v_b_e_L[i] - data->m1->wind[i];
		o->wind[i] = data->m1->wind[i];
	}
	map_vector ( data->dcm_lb, v_b_a_L, v_b_a_B );

	o->tas = sqrt ( SQ ( v_b_a_B[0] ) + SQ ( v_b_a_B[1] ) +
									SQ ( v_b_a_B[2] ) );
	o->cas = sqrt ( gi->traj->nav->densityr ) * o->tas;

	if ( o->tas > 1.0 )
	{
		data->alpha = atan2 ( v_b_a_B[2], v_b_a_B[0] );
		data->beta = atan2 ( v_b_a_B[1],
												 sqrt ( SQ ( v_b_a_B[0] ) + SQ ( v_b_a_B[2] ) ) );
	}
	else
	{
		data->alpha = data->beta = 0.0;
	}
	cal = cos ( data->alpha );
	sal = sin ( data->alpha );
	cbe = cos ( data->beta );
	sbe = sin ( data->beta );

	/* fuel */

/*if( mo->mass->fuelMax > 0.01 )
	o->fuel = st->fuel/mo->mass->fuelMax;
else*/
	o->fuel = 1;

	o->alpha = data->alpha * C_RAD2DEG;
	o->beta = data->beta * C_RAD2DEG;
	o->tas *= C_FPS2KT;
	o->cas *= C_FPS2KT;

	/* velocity wrt Earth */

	o->vs = -v_b_e_L[2] * 60.0;
	o->gs = sqrt ( SQ ( v_b_e_L[0] ) + SQ ( v_b_e_L[1] ) );
	if ( sqrt ( SQ ( o->gs ) + SQ ( v_b_e_L[2] ) ) > 1.0 )
	{
		o->track = atan2 ( v_b_e_L[1], v_b_e_L[0] );
		o->gamma = -atan2 ( v_b_e_L[2], o->gs );
	}
	else
	{
		o->track = psi;
		o->gamma = theta;
	}
	o->gs *= C_FPS2KT;
	o->track *= C_RAD2DEG;
	o->gamma *= C_RAD2DEG;

	/* at least put the bias in */

	o->a1 = motionControls.pitchCycTrim * C_RAD2DEG;
	o->b1 = motionControls.rollCycTrim * C_RAD2DEG;

	/* actuators */

	o->delm[0] = data->m1->delm[0];
	o->delm[1] = data->m1->delm[1];
	o->delm[2] = data->m1->delm[2];
	o->delf[0] = data->m1->delf[0];
	o->delt[0] = data->m1->delt[0];

	/* mode stuff */

	gi->traj->nav->controlType = gi->set->controlType;

	/* load factor */

	o->G = data->m1->G;

	/* rpm */

	if ( data->m1->rpm > set->maxRPM )
	{
		o->rpm = 0;
	}
	else
	{
		o->rpm = data->m1->rpm;
	}

	/* landing gear */

	o->wow = MIN ( data->m0->wow, 1 );
	gi->traj->nav->wow = o->wow;

	/* get wind axis system */

	dcm_wb[0][0] = cal * cbe;
	dcm_wb[0][1] = -cal * sbe;
	dcm_wb[0][2] = -sal;
	dcm_wb[1][0] = sbe;
	dcm_wb[1][1] = cbe;
	dcm_wb[1][2] = 0.0;
	dcm_wb[2][0] = sal * cbe;
	dcm_wb[2][1] = -sal * sbe;
	dcm_wb[2][2] = cal;
	matrix_multiply ( data->dcm_bl, dcm_wb, data->dcm_wl );
	matrix_transpose ( data->dcm_wl, data->dcm_lw );
	dcm2euler ( data->dcm_wl, &o->phiw, &o->thetaw, &o->psiw );

	o->phiw *= C_RAD2DEG;
	o->thetaw *= C_RAD2DEG;
	o->psiw *= C_RAD2DEG;

	dcm90[0][0] = -data->dcm_wl[0][2];
	dcm90[0][1] = data->dcm_wl[0][1];
	dcm90[0][2] = data->dcm_wl[0][0];
	dcm90[1][0] = -data->dcm_wl[1][2];
	dcm90[1][1] = data->dcm_wl[1][1];
	dcm90[1][2] = data->dcm_wl[1][0];
	dcm90[2][0] = -data->dcm_wl[2][2];
	dcm90[2][1] = data->dcm_wl[2][1];
	dcm90[2][2] = data->dcm_wl[2][0];
	dcm2euler ( dcm90, &o->phiw90, &o->thetaw90, &o->psiw90 );
	o->phiw90 *= C_RAD2DEG;
	o->thetaw90 *= C_RAD2DEG;
	o->psiw90 *= C_RAD2DEG;

	saveFloat ( data->dcm_wl, o->float_dcm_wl );
	saveFloat ( data->dcm_lw, o->float_dcm_lw );

	/* extrapolate command itself */
	if ( gi->datalink->m1->lostComm ) fp = gi->traj->flightPlanLCDL;
	else                             fp = gi->traj->flightPlanDL;


}

static void updateOneGCS ( struct gcs_ref* g, struct gcsInstance_ref* gi,
													 unsigned char theActiveInstance )
{

	struct gcsDatalink_ref* data = gi->datalink;

	char buttonCommand[32];

	if ( !gi->run )
	{
		return;
	}

	switch ( gi->gcsType )
	{

		case GUST_INTERFACE:
		default:

		if ( sim.mode == SIM_MODE_INIT )
		{
			gi->datalink->time = 0;
		}

		/* and now for the datalink... */
		readDatalink ( g, gi, data->p1, &data->datalinkStatus1, theActiveInstance );
		readDatalink ( g, gi, data->p2, &data->datalinkStatus2, theActiveInstance );

		/* take care of panel states */
		readPanelStates ( g, gi );

		// Joystick input updates - only update if this is the active instance to avoid messing with
		//  toggles
		if ( theActiveInstance == 1 )
		{
			updateJoyInputs ( gi->cntrlInput, sim.time );

			// Handle any GCS/Onboard functions that have special input logic here
			// Lock the motor start
			if ( ( gi->traj->lockMotorStart ) && ( gi->cntrlInput->safeOff->output == 1 ) )
			{
				gcsAddLine ( gi, "Motor Start Not Allowed, GCS Safed" );
				sceneMessageAll ( "Motor Start Not Allowed, GCS Safed" );
				gi->cntrlInput->safeOff->output = 0;
			}

			// Lock air mode
			if ( ( gi->traj->lockMotorStart ) && ( gi->cntrlInput->air->output == 1 ) )
			{
				gcsAddLine ( gi, "Motor Start Not Allowed, GCS Safed" );
				sceneMessageAll ( "Motor Start Not Allowed, GCS Safed" );
				gi->cntrlInput->air->output = 0;
			}

			// Video toggle
			if ( gi->cntrlInput->videoToggle->output == 1 ) // Switch was activated
			{
				gi->cntrlInput->videoToggle->output = -1;  /* swap video source */
			}

			// Safe off
			if ( 1 == gi->set->waitForNav )
			{
				if ( 0 == gi->datalink->m0->navStatus || 2 == gi->datalink->m0->navStatus )
				{
					// Safe off must be set to FALSE in this case
					gi->cntrlInput->safeOff->output = 0;
				}
			}

			// Change the motor lock
			if ( gi->cntrlInput->systemSafety->output == 1 )
			{
				if ( gi->traj->lockMotorStart == 1 )
				{
					commandExecute ( "systemSafety off" );
				}
				else
				{
					commandExecute ( "systemSafety on" );
				}
			}

			if ( ( data->lastWowUp != ( gi->cntrlInput->air->output || gi->cntrlInput->airTakeoff->output ) ) &&
					 ( gi->cntrlInput->air->output || gi->cntrlInput->airTakeoff->output ) )
			{
				if ( gi->cntrlInput->airTakeoff->output )
				{
					gcsAddLine ( gi, "AIR mode with takeoff selected" );
				}
				else
				{
					gcsAddLine ( gi, "AIR mode selected" );
				}
			}
			data->lastWowUp = ( gi->cntrlInput->air->output || gi->cntrlInput->airTakeoff->output );

			if ( data->lastWowDown != ( gi->cntrlInput->ground->output || gi->cntrlInput->groundLanding->output ) &&
					 ( gi->cntrlInput->ground->output || gi->cntrlInput->groundLanding->output ) )
			{
				if ( gi->cntrlInput->groundLanding->output )
				{
					gcsAddLine ( gi, "GROUND mode with landing selected" );
				}
				else
				{
					gcsAddLine ( gi, "GROUND mode selected" );
				}
			}
			data->lastWowDown = ( gi->cntrlInput->ground->output || gi->cntrlInput->groundLanding->output );

			if ( ( data->lastSafeOff != gi->cntrlInput->safeOff->output ) && ( gi->cntrlInput->safeOff->output == 1 ) )
			{
				gcsAddLine ( gi, "augment on (rotor spin)" );
			}
			data->lastSafeOff = gi->cntrlInput->safeOff->output;

			// Note: Once everything switches over to the input mapping, this will no longer be necessary (the check for change)
			//  since the input mapping handles that.
			if ( ( data->lastTogglePlanButton != gi->cntrlInput->planToggle->output ) && ( gi->cntrlInput->planToggle->output == 1 ) )
			{
				commandExecute ( "trajEdit" );
			}
			data->lastTogglePlanButton = gi->cntrlInput->planToggle->output;

			if ( sim.time > ( data->lastLoadRun + data->updateDtLoadRun ) )
			{
				if ( gi->traj->uploadEach[gi->traj->edit] )
				{
					if ( ( gi->cntrlInput->loadRunPlan->output == 1 ) || ( gi->cntrlInput->loadRunPlan->secRemaining > 0.0 ) )
					{
						// Immediately upload when the button combination is correctly pressed; only the trajGo happens after
						//  any defined time delay
						commandExecute ( "trajUpload" );
						data->lastLoadRun = sim.time;
					}
				}
				else if ( gi->cntrlInput->loadRunPlan->output == 1 )
				{
					commandExecute ( "trajGo" );
					data->lastLoadRun = sim.time;
				}
				// else: only these two cases, the else if combines the else and separate if checks
			}

			// Auto takeoff - should only be used when we can infer wow or have a sonar or LRF
			if ( ( gi->cntrlInput->airTakeoff->output == 1 ) && ( data->m0->wow ) )
			{
				// Only take off if we are on the ground
				// Always execute the trajStop first to ensure that the trajMove command is obeyed
				char command[29];
				sprintf ( command, "trajMove up %f sudo\0", gi->cntrlInput->joystickAutoTakeoffAlt );
				commandExecute ( command );
			}

			if ( ( gi->cntrlInput->groundLanding->output == 1 ) && ( data->m0->wow == 0 ) )
			{
				// Only land if we are in the air
				// Always execute the trajStop first to ensure that the trajMove command is obeyed
				commandExecute ( "trajMove land sudo" );
			}

			if ( ( gi->cntrlInput->rtb->output == 1 ) && ( data->m0->wow == 0 ) )
			{
				// Only return to base if we are in the air
				// Always execute the trajStop first to ensure that the trajMove command is obeyed
				commandExecute ( "trajMove rtb sudo" );
			}

			if ( gi->cntrlInput->stopPlan->output == 1 )
			{
				// Stop the current plan
				commandExecute ( "trajStop" );
			}

			// Check for panic button console presses
			if ( gi->cntrlInput->yellowButton->output == 1 )
			{
				// execute the specified input script
				sprintf ( buttonCommand, "%s", gi->cntrlInput->yellowButton->name );
				commandExecute ( buttonCommand );

			}

			// Check for panic button console presses
			if ( gi->cntrlInput->whiteButton->output == 1 )
			{
				sprintf ( buttonCommand, "%s", gi->cntrlInput->whiteButton->name );
				// execute the specified input script
				commandExecute ( buttonCommand );

			}

			// Check for panic button console presses
			if ( gi->cntrlInput->blueButton->output == 1 )
			{
				sprintf ( buttonCommand, "%s", gi->cntrlInput->blueButton->name );
				// execute the specified input script
				commandExecute ( buttonCommand );

			}

			// Check for panic button console presses
			if ( gi->cntrlInput->greenButton->output == 1 )
			{
				sprintf ( buttonCommand, "%s", gi->cntrlInput->greenButton->name );
				// execute the specified input script
				commandExecute ( buttonCommand );

			}

			// Check for panic button console presses
			if ( gi->cntrlInput->bigRedButton->output == 1 )
			{
				sprintf ( buttonCommand, "%s", gi->cntrlInput->bigRedButton->name );
				// execute the specified input script
				commandExecute ( buttonCommand );

			}
		}

		/* send uplink messages */

		/* standard joystick messsage */

		/* I'm leaving this in the instance code even though only one of them does it just to keep all the "sends" in the same place */

		if ( data->enable0 == 0 || sim.mode == SIM_MODE_INIT || theActiveInstance == 0 )
		{ /* only the active instance can send joystick stuff! */
			data->lastUpdate0 = sim.time;
			data->lastLoadRun = 0;
		}
		else
		{
			if ( sim.time >= data->lastUpdate0 + data->updateDt0 )
			{
				// Analog functions
				data->up0->throttleLever = ( float ) ( gi->cntrlInput->throttle->output );
				data->up0->pitchStick = ( float ) ( gi->cntrlInput->pitch->output );
				data->up0->rollStick = ( float ) ( gi->cntrlInput->roll->output );
				data->up0->rudderPedal = ( float ) ( gi->cntrlInput->rudder->output );
				doUp0buttons ( gi );

				/* check for need to blend with other inputs */
				if ( sim.time >= data->lastUpdateOther0 && sim.time < data->lastUpdateOther0 + data->otherTimeout )
				{
					data->up0->throttleLever = doStickBlend ( data->up0->throttleLever, data->lastThrottleLeverOther );
					data->up0->rollStick = doStickBlend ( data->up0->rollStick, data->lastRollStickOther );
					data->up0->pitchStick = doStickBlend ( data->up0->pitchStick, data->lastPitchStickOther );
					data->up0->rudderPedal = doStickBlend ( data->up0->rudderPedal, data->lastRudderPedalOther );
					data->up0->button[10] = doButtonBlend ( data->up0->button[10], data->lastButton10Other );
					data->up0->button[15] = doButtonBlend ( data->up0->button[15], data->lastButton15Other );
				}

				/* encode message */
				data->up0->messageID = DATALINK_MESSAGE_UP0;
				data->up0->messageSize = sizeof ( struct datalinkMessageUp0_ref );

				/* send message */
				datalinkCheckSumEncode ( ( unsigned char* ) data->up0, sizeof ( struct datalinkMessageUp0_ref ) );
				writePort ( data->p1, ( char* ) data->up0, sizeof ( struct datalinkMessageUp0_ref ) );
				writePort ( data->p2, ( char* ) data->up0, sizeof ( struct datalinkMessageUp0_ref ) );
				data->lastUplink = sim.time;

				/* copy joystick info for trajectory generation purposes on gcs */
				gi->traj->joy->rollStick = data->up0->rollStick;
				gi->traj->joy->pitchStick = data->up0->pitchStick;
				gi->traj->joy->rudderPedal = data->up0->rudderPedal;
				gi->traj->joy->throttleLever = data->up0->throttleLever;
				gi->traj->joy->goFastButton = data->up0->button[10];

				/* deal with counters */
				data->lastUpdate0 += data->updateDt0; /* sloppy */
			}
		}

		/* heartbeat message */

		if ( sim.mode == SIM_MODE_INIT )
		{

			data->lastUplink = -data->updateDtHeartbeat;

		}
		else if ( data->enableHeartbeat && sim.time >= data->lastUplink + data->updateDtHeartbeat )
		{

			/* encode message */

			data->heartbeat->messageID = DATALINK_MESSAGE_HEARTBEAT;
			data->heartbeat->messageSize = sizeof ( struct datalinkHeader_ref );

			/* send message */
			datalinkCheckSumEncode ( ( unsigned char* ) data->heartbeat, sizeof ( struct datalinkHeader_ref ) );
			writePort ( data->p1, ( char* ) data->heartbeat, sizeof ( struct datalinkHeader_ref ) );
			writePort ( data->p2, ( char* ) data->heartbeat, sizeof ( struct datalinkHeader_ref ) );   //recieving in qt-app

			data->lastUplink = sim.time;

		}

		/* compute outputs for scene */

		gcsOutputUpdate ( g, gi, theActiveInstance );

		break;
	}

}


void updateGCS ( void )
{

	struct gcs_ref* g = &gcs;

	/* enforce keeping datums consistant if GCS is operating, for simulator */
//(might want to turn it off in ship-heli simulation)
	if ( vehicleSet.enforceDatumWithNav )
	{
		vehicleSet.refLatitude = navinit.datumLat;
		vehicleSet.refLongitude = navinit.datumLon;
		vehicleSet.datumAlt = navinit.datumAlt;
	}
	/* make sure active instance is running */

	if ( !( gcsActiveInstance ( g )->run ) )
	{
		if ( g->instance0->run ) commandExecute ( "gcsSelect 0" );
	}

	/* update all the instances */

	updateOneGCS ( g, g->instance0, ( g->active == 0 ) );

}

void motorStart ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;
	char buffer[200];

	if ( checkSystemSafety () == 0 )
	{
		gcsAddLine ( gi, "Motor Start Not Allowed, GCS Safed" );
		sceneMessageAll ( "Motor Start Not Allowed, GCS Safed" );
		return;
	}

	if ( 1 == gi->set->waitForNav )
	{
		if ( 0 == gi->datalink->m0->navStatus || 2 == gi->datalink->m0->navStatus || 6 == gi->datalink->m0->navStatus )
		{
			/* nav is on BIT, initializing, or failed */
			gcsAddLine ( gi, "Nav not ready" );
			sceneMessageAll ( "Nav not ready" );
			return;
		}
	}

	if ( argc > 0 )
	{
		if ( !strcmp ( argv[argc - 1], "test" ) )
		{
			data->trajectory->command = TRAJECTORY_MOTOR_SPIN;
		}
		else
		{
			data->trajectory->command = TRAJECTORY_MOTOR_START;
		}
	}
	else
	{
		data->trajectory->command = TRAJECTORY_MOTOR_START;
	}

	/* encode message */
	data->trajectory->messageID = DATALINK_MESSAGE_TRAJECTORY;
	data->trajectory->messageSize = sizeof ( struct datalinkMessageTrajectory_ref );
	data->trajectory->spare++;

	/* send message */
	datalinkCheckSumEncode ( ( unsigned char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p1, ( char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p2, ( char* ) data->trajectory, data->trajectory->messageSize );
	data->lastUplink = sim.time;

	sprintf ( buffer, "motor start message sent to " );
	gcsAddVehicleName ( buffer, gi->outputs );
	gcsAddLine ( gi, buffer );

}

void motorStop ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;
	char buffer[200];

	data->trajectory->command = TRAJECTORY_MOTOR_STOP;

	/* encode message */
	data->trajectory->messageID = DATALINK_MESSAGE_TRAJECTORY;
	data->trajectory->messageSize = sizeof ( struct datalinkMessageTrajectory_ref );
	data->trajectory->spare++;

	/* send message */
	datalinkCheckSumEncode ( ( unsigned char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p1, ( char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p2, ( char* ) data->trajectory, data->trajectory->messageSize );
	data->lastUplink = sim.time;

	sprintf ( buffer, "motor stop message sent to " );
	gcsAddVehicleName ( buffer, gi->outputs );
	gcsAddLine ( gi, buffer );

}

void vehicleArm ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;
	char buffer[200];
	int numArgs = argc;

	data->trajectory->command = TRAJECTORY_ARM;

	if ( argc > 0 )
	{
		// See if the override "super" command was sent
		if ( !strcmp ( argv[argc - 1], "sudo" ) )
		{
			// Add the override - the flag in the waypoint field.  The flag is 255 to set several bits of data.
			//  Be careful with this override - it bypasses air/ground checks, allowing a disarm/arm to happen in air.
			data->trajectory->waypoint = TRAJECTORY_COMMAND_CHECKS_OVERRIDE;
		}
	}

	/* encode message */
	data->trajectory->messageID = DATALINK_MESSAGE_TRAJECTORY;
	data->trajectory->messageSize = sizeof ( struct datalinkMessageTrajectory_ref );
	data->trajectory->spare++;

	/* send message */
	datalinkCheckSumEncode ( ( unsigned char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p1, ( char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p2, ( char* ) data->trajectory, data->trajectory->messageSize );
	data->lastUplink = sim.time;

	sprintf ( buffer, "vehicle ARM message sent to " );
	gcsAddVehicleName ( buffer, gi->outputs );
	gcsAddLine ( gi, buffer );

}

void vehicleDisarm ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;
	char buffer[200];
	int numArgs = argc;

	data->trajectory->command = TRAJECTORY_DISARM;

	if ( argc > 0 )
	{
		// See if the override "super" command was sent
		if ( !strcmp ( argv[argc - 1], "sudo" ) )
		{
			// Add the override - the flag in the waypoint field.  The flag is 255 to set several bits of data.
			//  Be careful with this override - it bypasses air/ground checks, allowing a disarm/arm to happen in air.
			data->trajectory->waypoint = TRAJECTORY_COMMAND_CHECKS_OVERRIDE;
		}
	}

	/* encode message */
	data->trajectory->messageID = DATALINK_MESSAGE_TRAJECTORY;
	data->trajectory->messageSize = sizeof ( struct datalinkMessageTrajectory_ref );
	data->trajectory->spare++;

	/* send message */
	datalinkCheckSumEncode ( ( unsigned char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p1, ( char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p2, ( char* ) data->trajectory, data->trajectory->messageSize );
	data->lastUplink = sim.time;

	sprintf ( buffer, "vehicle DISARM message sent to " );
	gcsAddVehicleName ( buffer, gi->outputs );
	gcsAddLine ( gi, buffer );

}

int fcpSpecificWrite ( char* arg )
{
	if ( !strcmp ( arg, "version" ) )
	{
		return 1;
	}
	else if ( !strcmp ( arg, "pwms1" ) )
	{
		return 2;
	}
	else if ( !strcmp ( arg, "pwms2" ) )
	{
		return 3;
	}
	else if ( !strcmp ( arg, "pwms3" ) )
	{
		return 4;
	}
	else if ( !strcmp ( arg, "pwms4" ) )
	{
		return 5;
	}
	else if ( !strcmp ( arg, "radio" ) )
	{
		return 6;
	}
	else if ( !strcmp ( arg, "sensor" ) )
	{
		return 7;
	}
	else if ( !strcmp ( arg, "sas" ) )
	{
		return 8;
	}
	else if ( !strcmp ( arg, "rpm" ) )
	{
		return 9;
	}
	else if ( !strcmp ( arg, "mixerGen" ) )
	{
		return 10;
	}
	else if ( !strcmp ( arg, "mixerSpec" ) )
	{
		return 11;
	}
	else
	{
		return -1;
	}
}

void wowOff ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;
	char buffer[200];

	if ( checkSystemSafety () == 0 )
	{
		gcsAddLine ( gi, "Motor Start Not Allowed, GCS Safed" );
		sceneMessageAll ( "Motor Start Not Allowed, GCS Safed" );
		return;
	}


	data->trajectory->command = TRAJECTORY_WOW_OFF;

	/* encode message */
	data->trajectory->messageID = DATALINK_MESSAGE_TRAJECTORY;
	data->trajectory->messageSize = sizeof ( struct datalinkMessageTrajectory_ref );
	data->trajectory->spare++;

	/* send message */
	datalinkCheckSumEncode ( ( unsigned char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p1, ( char* ) data->trajectory, data->trajectory->messageSize );
	writePort ( data->p2, ( char* ) data->trajectory, data->trajectory->messageSize );
	data->lastUplink = sim.time;

	sprintf ( buffer, "AIR mode message sent to " );
	gcsAddVehicleName ( buffer, gi->outputs );
	gcsAddLine ( gi, buffer );

}


void wowOn ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;
	char buffer[200];

	if ( data->m0->motor == 0 || gi->cntrlInput->arm->output || data->m0->wow )
	{

		data->trajectory->command = TRAJECTORY_WOW_ON;

		/* encode message */
		data->trajectory->messageID = DATALINK_MESSAGE_TRAJECTORY;
		data->trajectory->messageSize = sizeof ( struct datalinkMessageTrajectory_ref );
		data->trajectory->spare++;

		/* send message */
		datalinkCheckSumEncode ( ( unsigned char* ) data->trajectory, data->trajectory->messageSize );
		writePort ( data->p1, ( char* ) data->trajectory, data->trajectory->messageSize );
		writePort ( data->p2, ( char* ) data->trajectory, data->trajectory->messageSize );
		data->lastUplink = sim.time;

		sprintf ( buffer, "GROUND mode message sent to " );
		gcsAddVehicleName ( buffer, gi->outputs );
		gcsAddLine ( gi, buffer );

	}
	else
	{
		gcsAddLine ( gi, "GROUND mode needs to be armed" );
	}

}


void gpsDenied ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;
	char sendMessage = 1;
	char buffer[200];

	sprintf ( buffer, " " ); /* just to prevent a case where buffer isn't set */

	if ( argc == 2 )
	{
		if ( !strcmp ( argv[1], "on" ) )
		{
			data->trajectory->command = TRAJECTORY_GPSDENIED_ON;
			sprintf ( buffer, "GPS denied mode ON message sent to " );
		}
		else if ( !strcmp ( argv[1], "gpson" ) )
		{
			data->trajectory->command = TRAJECTORY_GPSDENIED_GPSON;
			sprintf ( buffer, "GPS denied mode GPSON message sent to " );
		}
		else if ( !strcmp ( argv[1], "off" ) )
		{
			data->trajectory->command = TRAJECTORY_GPSDENIED_OFF;
			sprintf ( buffer, "GPS denied mode OFF message sent to " );
		}
		else
		{
			sendMessage = 0;
		}
	}
	else
	{
		sendMessage = 0;
	}

	if ( sendMessage )
	{
		/* encode message */
		data->trajectory->messageID = DATALINK_MESSAGE_TRAJECTORY;
		data->trajectory->messageSize = sizeof ( struct datalinkMessageTrajectory_ref );
		data->trajectory->spare++;

		/* send message */
		datalinkCheckSumEncode ( ( unsigned char* ) data->trajectory, data->trajectory->messageSize );
		writePort ( data->p1, ( char* ) data->trajectory, data->trajectory->messageSize );
		writePort ( data->p2, ( char* ) data->trajectory, data->trajectory->messageSize );
		data->lastUplink = sim.time;

		gcsAddVehicleName ( buffer, gi->outputs );
		gcsAddLine ( gi, buffer );

	}
	else
	{
		gcsAddLine ( gi, "gpsDenied [on/gpson/off]" );
	}

}

void getDir ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;

	Dir* found;
	char buffer[120], name[80];
	int dirNum = 0, size;
	void* dataPtr;
	float updateDt = 0.0;

	if ( argc > 1 )
	{
		if ( !strcmp ( argv[1], "stop" ) )
		{

			/* put data in the message */
			data->getdir->dirNum = -1;
			data->getdir->uniqueID++;
			data->getdir->updateDt = -1;

			/* encode message */
			data->getdir->messageID = DATALINK_MESSAGE_GETDIR;
			data->getdir->messageSize = sizeof ( struct datalinkMessageGetdir_ref );

			/* send message */
			datalinkCheckSumEncode ( ( unsigned char* ) data->getdir, data->getdir->messageSize );
			writePort ( data->p1, ( char* ) data->getdir, data->getdir->messageSize );
			writePort ( data->p2, ( char* ) data->getdir, data->getdir->messageSize );
			data->lastUplink = sim.time;
			return;

		}
		else
		{
			strcpy ( name, argv[1] );
			if ( argc > 2 )
			{
				updateDt = ( float ) atof ( argv[2] );
			}
		}
	}
	else
	{
		strcpy ( name, ( char* ) ( brwin[LIMIT ( brwins.currentWin, 0, 2 )].currentDir ) );
	}

	found = findDir ( name );
	if ( found != NULL )
	{

		/* find dirNum that corresponds to requested dir */
		do
		{
			dataPtr = getDirDataPointer ( dirNum, &size );
			dirNum++;
		}
		while ( dataPtr != found->data && dataPtr != NULL );

		if ( size == 0 )
		{
			if ( dataPtr == NULL )
				sprintf ( buffer, "'%s' not enabled for get/send", name );
			else
				sprintf ( buffer, "'%s' is too big for get/send", name );
		}
		else
		{

			/* put data in the message */
			data->getdir->dirNum = dirNum - 1;
			data->getdir->uniqueID++;
			data->getdir->updateDt = updateDt;

			/* encode message */
			data->getdir->messageID = DATALINK_MESSAGE_GETDIR;
			data->getdir->messageSize = sizeof ( struct datalinkMessageGetdir_ref );

			/* send message */
			datalinkCheckSumEncode ( ( unsigned char* ) data->getdir, data->getdir->messageSize );
			writePort ( data->p1, ( char* ) data->getdir, data->getdir->messageSize );
			writePort ( data->p2, ( char* ) data->getdir, data->getdir->messageSize );
			data->lastUplink = sim.time;

			if ( updateDt > 0.0 )
			{
				sprintf ( buffer, "'%s' get every %.2f sec request sent to ", name, updateDt );
			}
			else
			{
				sprintf ( buffer, "'%s' get request sent to ", name );
			}
			gcsAddVehicleName ( buffer, gi->outputs );
		}

	}
	else
	{
		sprintf ( buffer, "'%s' directory not found", name );
	}

	gcsAddLine ( gi, buffer );

}


/*void getDirButton( int argc, char **argv ) {

	char *argv2[2] = {NULL,(char *)(brwin[0].currentDir)};

	getDir( 2, argv2 );

}*/

void sendDirToInstance ( struct gcsInstance_ref* gi, char* dirName )
{
	struct gcsDatalink_ref* data = gi->datalink;

	Dir* found;
	char buffer[120];
	int dirNum = 0, size;
	void* dataPtr;

	found = findDir ( dirName );
	if ( found != NULL )
	{

		/* find dirNum that corresponds to requested dir */

		do
		{
			dataPtr = getDirDataPointer ( dirNum, &size );
			dirNum++;
		}
		while ( dataPtr != found->data && size > 0 );

		if ( size == 0 )
		{
			sprintf ( buffer, "'%s' sendDir not successful", dirName );
		}
		else
		{

			memcpy ( data->senddir->data, dataPtr, size );
			data->senddir->dirNum = dirNum - 1;
			data->senddir->size = size;
			data->senddir->uniqueID++;
			data->senddir->verbose = 1;

			/* encode message */

			data->senddir->messageID = DATALINK_MESSAGE_SENDDIR;
			data->senddir->messageSize = SENDDIR_BASESIZE + size;

			/* send message */
			datalinkCheckSumEncode ( ( unsigned char* ) data->senddir, data->senddir->messageSize );
			writePort ( data->p1, ( char* ) data->senddir, data->senddir->messageSize );
			writePort ( data->p2, ( char* ) data->senddir, data->senddir->messageSize );
			data->lastUplink = sim.time;

			sprintf ( buffer, "'%s' sent to ", dirName );
			gcsAddVehicleName ( buffer, gi->outputs );
		}

	}
	else
	{
		sprintf ( buffer, "'%s' directory not found", dirName );
	}

	gcsAddLine ( gi, buffer );

}
void sendDir ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi = gcsActiveInstance ( g );
	struct gcsDatalink_ref* data = gi->datalink;

	Dir* found;
	char buffer[120];
	int dirNum = 0, size;
	void* dataPtr;

	if ( argc > 1 )
	{
		found = findDir ( argv[1] );
		if ( found != NULL )
		{

			/* find dirNum that corresponds to requested dir */

			do
			{
				dataPtr = getDirDataPointer ( dirNum, &size );
				dirNum++;
			}
			while ( dataPtr != found->data && size > 0 );

			if ( size == 0 )
			{
				sprintf ( buffer, "'%s' sendDir not successful", argv[1] );
			}
			else
			{

				memcpy ( data->senddir->data, dataPtr, size );
				data->senddir->dirNum = dirNum - 1;
				data->senddir->size = size;
				data->senddir->uniqueID++;
				data->senddir->verbose = 1;

				/* encode message */

				data->senddir->messageID = DATALINK_MESSAGE_SENDDIR;
				data->senddir->messageSize = SENDDIR_BASESIZE + size;

				/* send message */
				datalinkCheckSumEncode ( ( unsigned char* ) data->senddir, data->senddir->messageSize );
				writePort ( data->p1, ( char* ) data->senddir, data->senddir->messageSize );
				writePort ( data->p2, ( char* ) data->senddir, data->senddir->messageSize );
				data->lastUplink = sim.time;

				sprintf ( buffer, "'%s' sent to ", argv[1] );
				gcsAddVehicleName ( buffer, gi->outputs );
			}

		}
		else
		{
			sprintf ( buffer, "'%s' directory not found", argv[1] );
		}
	}
	else
		sprintf ( buffer, "usage: sendDir <dirname>" );

	gcsAddLine ( gi, buffer );

}

#define SERVOMAP_START         0
#define SERVOMAP_DOWN_NEUTRAL  1
#define SERVOMAP_DOWN_RIGHT    2
#define SERVOMAP_DOWN_LEFT     3
#define SERVOMAP_DOWN_DOWN     4
#define SERVOMAP_DOWN_UP       5
#define SERVOMAP_DOWNR_NEUTRAL 6
#define SERVOMAP_DOWNL_NEUTRAL 7
#define SERVOMAP_UP_NEUTRAL    8
#define SERVOMAP_DONE          9


void commandDiagnostics ( int argc, char** argv )
{

	if ( argc == 2 )
	{
		if ( !strcmp ( argv[1], "on" ) )
		{
			commandExecute ( "rc diagnostics.on = 1" );
		}
		else if ( !strcmp ( argv[1], "off" ) )
		{
			commandExecute ( "rc diagnostics.on = 0" );
		}
		else if ( !strcmp ( argv[1], "reset" ) )
		{
			commandExecute ( "rc diagnostics.n = 0" );
		}
	}
	else
	{
		gcsAddLine ( NULL, "usage: diagnostics [on|off|reset]" );
	}

}


void commandSystemSafety ( int argc, char** argv )
{
	struct gcsInstance_ref* gi = gcsActiveInstance ( &gcs );
	struct gcsDatalink_ref* data = gi->datalink;
	if ( argc == 2 )
	{
		if ( !strcmp ( argv[1], "on" ) )
		{
			// Set the value of the safety flag
			if ( ( data->m0->wow ) && ( data->m0->motor == 0 ) )
			{
				gi->traj->lockMotorStart = 1;
			}
		}
		else if ( !strcmp ( argv[1], "off" ) )
		{
			// Set the value of the safety flag
			if ( ( data->m0->wow ) && ( data->m0->motor == 0 ) )
			{
				gi->traj->lockMotorStart = 0;
			}
		}
		else
		{
			gcsAddLine ( NULL, "usage: systemSafety [on|off]" );
		}
	}
	else
	{
		gcsAddLine ( NULL, "usage: systemSafety [on|off]" );
	}
}


void commandGcsSelect ( int argc, char** argv )
{

	struct gcs_ref* g = &gcs;
	struct gcsInstance_ref* gi;
	char buffer[200];
	char oldActive = g->active;
	int i;

	if ( gcsActiveInstance ( &gcs )->datalink->uploadingPlan ) return;

	if ( argc == 2 )
	{
		if ( !strcmp ( argv[1], "0" ) )
		{
			g->active = 0;
		}
		else if ( !strcmp ( argv[1], "1" ) )
		{
			g->active = 1;
		}
		else if ( !strcmp ( argv[1], "2" ) )
		{
			g->active = 2;
		}
		else if ( !strcmp ( argv[1], "3" ) )
		{
			g->active = 3;
		}
		else
		{
			gcsAddLine ( NULL, "usage: gcsSelect [0|1|2|4]" );
		}
	}
	else
	{
		g->active = 0;
	}

	if ( g->active != oldActive )
	{ /* blow away any popups */
		struct cnslFileNamePopup_ref* fn = &cnslFileNamePopup;
		struct cnslConfirmPopup_ref* c = &cnslConfirmPopup;
		if ( fn->open ) glutDestroyWindow ( fn->win );
		fn->open = 0;
		if ( c->open ) glutDestroyWindow ( c->win );
		c->open = 0;

		/* sort out the scratch pad flight plan */
		/* for legacy reasons (format of flight plan files) */
		/* workpad needs to know the number of waypoints, in case GCS not running */
		gi = gcsGetInstance ( g, oldActive );
		gi = gcsGetInstance ( g, g->active );

		/* now for the really wierd part.  Make global references that point to active directory */
		gcsSetReferences ();

		/* no longer selcting any waypoints */
		for ( i = 0; i < MAN_NMANS; i++ )
		{
			scene0.waySelected[i] = 0;
			scene1.waySelected[i] = 0;
			scene2.waySelected[i] = 0;
		}

		/* make sure it is on */
		gi->run = 1;

		/* we're going to need labels */
		if ( g->active > 0 && sceneGlobal.showNameText != 99 ) sceneGlobal.showNameText = 1;
		if ( g->active > 0 && sceneGlobal.drawLineToText != 99 ) sceneGlobal.drawLineToText = 1;

		/* need to be able to pick stuff if adding one */
		if ( g->active > 0 ) sceneGlobal.enableGCSSelect = 1;

		/* video source has probably changed */
		gi->cntrlInput->videoToggle->output = 99;

		/* send a nice message to scene */
		if ( gi->panel->inop )
		{
			sprintf ( buffer, "Slot %d, Inoperative ", g->active );
		}
		else
		{
			sprintf ( buffer, "Operating " );
		}
		gcsAddVehicleName ( buffer, gi->outputs );
		sceneMessageAll ( buffer );

	}

	sprintf ( buffer, "Now controlling [%d] ", g->active );
	gcsAddVehicleName ( buffer, gcsActiveInstance ( g )->outputs );
	gcsAddLine ( gcsActiveInstance ( g ), buffer );

}

/* command to allow comments to be added to the log file */
void commandRem ( int argc, char** argv )
{

	char message[127] = "";
	int i;

	if ( argc > 1 )
	{
		for ( i = 1; i < argc; i++ )
		{ /* append label(s) */
			strcat ( message, argv[i] );
			strcat ( message, " " );
		}
	}

	logInfo ( message );

	/*
	char toAircraft[256];

	strcat( toAircraft, "rc rem ", message );
	commandExecute( toAircraft );
	strcat( toAircraft, "rc2 rem ", message );
	commandExecute( toAircraft );
	*/
	return;

}


void initGCS ( void )
{

	char in[3][BSIZE][BSIZE] = { /* trajUpload */
				"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
			"bb                            bb",
			"b                              b",
			"b                              b",
			"b              b               b",
			"b             bbb              b",
			"b            bbbbb             b",
			"b           bbbbbbb            b",
			"b          bbbbbbbbb           b",
			"b         bbbbbbbbbbb          b",
			"b             bbb              b",
			"b             bbb              b",
			"b             bbb              b",
			"b             bbb              b",
			"b          bb bbb bb           b",
			"b         b   bbb   b          b",
			"b         b   bbb   bb         b",
			"b         b   bbb   b b        b",
			"b         b         b  b       b",
			"b         b         b   b      b",
			"b         b         b    b     b",
			"b          bbbbbbbbb     b     b",
			"b                              b",
			"b                              b",
			"b  bbb bb  b  b bb  b   bb     b",
			"b   b  b b b  b b b b   b b    b",
			"b   b  bb  b  b bb  b   b b    b",
			"b   b  b b b  b b   b   b b    b",
			"b   b  b b  bb  b   bbb bb     b",
			"b                              b",
			"bb                            bb",
			"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
			/* getDir */
						"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
					"bb                            bb",
					"b                              b",
					"b                              b",
					"b                              b",
					"b    bbbbbbbbb                 b",
					"b    bbbbbbbbb                 b",
					"b    bwbwwwwwb                 b",
					"b    bbbbbbbbb  bbb            b",
					"b    bwbwwwwwb     bb          b",
					"b    bbbbbbbbb       b         b",
					"b                     b        b",
					"b                      b       b",
					"b                      b       b",
					"b                      b       b",
					"b      bbbbbbbbb    b  b       b",
					"b      bbbbbbbbb   bb b        b",
					"b      bwbwwwwwb  bbbb         b",
					"b      bbbbbbbbb   bb          b",
					"b      bwbwwwwwb    b          b",
					"b      bbbbbbbbb               b",
					"b                              b",
					"b                              b",
					"b                              b",
					"b   bb  bbbb bbb bb  bbb bb    b",
					"b  b    b     b  b b  b  b b   b",
					"b  b bb bbb   b  b b  b  bb    b",
					"b  b  b b     b  b b  b  b b   b",
					"b   bb  bbbb  b  bb  bbb b b   b",
					"b                              b",
					"bb                            bb",
					"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
					/* trajStop */
								"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
							"bb                            bb",
							"b                              b",
							"b          wwwwwwwwww          b",
							"b         wwbbbbbbbbww         b",
							"b        wwbbbbbbbbbbww        b",
							"b       wwbbbbbbbbbbbbww       b",
							"b      wwbbbbbbbbbbbbbbww      b",
							"b     wwbbbbbbbbbbbbbbbbww     b",
							"b     wbbbbbbbbbbbbbbbbbbw     b",
							"b     wbbwwbwwwbbwbbwwwbbw     b",
							"b     wbwbbbbwbbwbwbwbbwbw     b",
							"b     wbwwwbbwbbwbwbwwwbbw     b",
							"b     wbbbwbbwbbwbwbwbbbbw     b",
							"b     wbwwbbbwbbbwbbwbbbbw     b",
							"b     wbbbbbbbbbbbbbbbbbbw     b",
							"b     wwbbbbbbbbbbbbbbbbww     b",
							"b      wwbbbbbbbbbbbbbbww      b",
							"b       wwbbbbbbbbbbbbww       b",
							"b        wwbbbbbbbbbbww        b",
							"b         wwbbbbbbbbww         b",
							"b          wwwwwwwwww          b",
							"b                              b",
							"b                              b",
							"b  bbb bb    bb bbb  b  bb     b",
							"b   b  b b  b    b  b b b b    b",
							"b   b  bb    b   b  b b bb     b",
							"b   b  b b    b  b  b b b      b",
							"b   b  b b  bb   b   b  b      b",
							"b                              b",
							"bb                            bb",
							"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
	};

	addCnslButton ( "trajUpload", in[0] );
	addCnslButton ( "trajStop", in[2] );
	addCnslButton ( "getDir", in[1] );

	commandExecute ( "brwin0.commandF1=getDir" );
	commandExecute ( "brwin1.commandF1=getDir" );
	commandExecute ( "brwin2.commandF1=getDir" );

	commandLoad ( "gcsSelect", commandGcsSelect,
								"GCS: pick which vehicle (0,1,2)" );
	commandLoad ( "pointLatLon", pointLatLong,
								"GCS: point to latitude/longitude" );
	commandLoad ( "pointLatLong", pointLatLong,
								"GCS: point to latitude/longitude" );
	commandLoad ( "motorStart", motorStart,
								"GCS: start motor" );
	commandLoad ( "motorStop", motorStop,
								"GCS: shutdown motor" );
	commandLoad ( "wowOff", wowOff,
								"GCS: manual wow off" );
	commandLoad ( "wowOn", wowOn,
								"GCS: manual wow on" );
	commandLoad ( "gpsDenied", gpsDenied,
								"GCS: GPS denied mode" );

	commandLoad ( "simStartLocation", simStartLocationCmd,
								"simulator:  set simulation start location" );
	commandLoad ( "gpscalc", gpscalcCmd,
								"GCS: do lat/long -> x/y calc" );
	commandLoad ( "gpsref", tellGpsref,
								"GCS: send ascii data to gpsref" );
	commandLoad ( "gpsrefConfigure", gpsrefConfigure,
								"GCS: configure reference station" );
	commandLoad ( "gpsrefCalibrate", gpsrefCalibrate,
								"GCS: begin calibration of ref" );
	commandLoad ( "gpsrefMobile", gpsrefMobile,
								"GCS: use reference antenna to locate moving vehicle" );
	commandLoad ( "gpsrefAlign", gpsrefAlign,
								"GCS: use reference antenna to locate moving vehicle with ALIGN" );
	commandLoad ( "datumSet", gpsrefDatum,
								"GCS: set datum to current location" );

	commandLoad ( "rc", remoteCommand,
								"GCS: send (remote) command to onboard computer" );
	commandLoad ( "getDir", getDir,
								"GCS: request directory data from onboard computer" );
	commandLoad ( "sendDir", sendDir,
								"GCS: send directory data to onboard computer" );

	commandLoad ( "resetTimer", resetTimer,
								"GCS: reset flight timer, remove all previous flights" );
	commandLoad ( "timerReset", resetTimer,
								"GCS: reset flight timer, remove all previous flights" );
	commandLoad ( "diagnostics", commandDiagnostics,
								"GCS: control onboard diagnostics" );

	commandLoad ( "systemSafety", commandSystemSafety,
								"GCS: command system safety lock on/off" );

	commandLoad ( "rem", commandRem,
								"GCS: add a comment to the log file" );

	commandLoad ( "vehicleArm", vehicleArm,
								"GCS: arm vehicle" );
	commandLoad ( "vehicleDisarm", vehicleDisarm,
								"GCS: disarm vehicle" );

	//createWinTrackView(onboard2.winTrackView);
	//initVisionFormationView(onboard2.visionFormationView);

	gcsSetReferences ();

}


