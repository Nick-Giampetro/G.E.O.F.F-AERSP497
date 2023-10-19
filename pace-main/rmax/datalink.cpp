
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
	* $Id: datalink.cpp,v 1.359 2007-12-12 00:59:34 nrooz Exp $
	* main code that interfaces the flight computer and encoding of data packets
	* 1) retrieve and send variables from gcs to onboard code
	* 2) reading from secondary flight computer
	***/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include "esim/util.h"
#include "esim/quat.h"
#include "esim/cnsl.h"
#include "esim/command.h"
#include "esim/sim_ref.h"
#include "rmax/serial.h"
#include "rmax/logger.h"
#include "rmax/checksum.h"
#include "rmax/onboard_ref.h"
#include "rmax/dted_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/controller_ref.h"
#include "rmax/sensors_ref.h"
#include "rmax/realScene_ref.h"

#include "rmax/datalink.h"

#define ISFINITE _finite
unsigned char* datUploadSectorData = NULL;

/* enable directories to be sent/received */

void* getDirDataPointer( int dirNum, int* size )
{
	void* data;

	switch ( dirNum )
	{
		default:
		case 0:
		data = &obDatalinkHeader;
		*size = sizeof( struct datalinkHeader_ref );
		break;
		case 1:
		data = &obDatalinkMessage0;
		*size = sizeof( struct datalinkMessage0_ref );
		break;
		case 2:
		data = &obDatalinkMessage1;
		*size = sizeof( struct datalinkMessage1_ref );
		break;
		case 3: /* ***LAST ONE ***, just change this case # to provide room for more dirs */
		data = NULL;
		*size = 0;
		break;
	}

	if ( *size > SENDDIR_MAXDIRSIZE )
	{
		printf( "datalink: directory (%d) too big (%d bytes) for get/send\n", dirNum, *size );
		*size = 0;
	}
	else if ( *size == 0 )
	{
		printf( "datalink: directory not enabled for get/send\n" );
		data = NULL;
	}

	return data;

}

static void readDatalink( struct onboard_ref* ob, serialPort_ref* port, double* rxTime )
{
	struct obDatalink_ref* data = ob->datalink;
	struct navigation_ref* nav = ob->navigation;
	struct onboardControl_ref* con = ob->control;

	int index, done;
	unsigned char* bf;
	void* dataPtr;
	int size;
	char gcsPrintBuff[100];   // buffer for printing to the gcs console

	readPort( port );

	if ( port->dataSource == PORT_OFF ) return;

	done = 0;
	index = 0;

	while ( ( index <= port->bytesread - ( int ) sizeof( struct datalinkHeader_ref ) ) && !done )
	{
		if ( ( port->buffer[index] == DATALINK_SYNC0 ) &&
				 ( port->buffer[index + 1] == DATALINK_SYNC1 ) &&
				 ( port->buffer[index + 2] == DATALINK_SYNC2 ) )
		{
			bf = &( port->buffer[index] );

			memcpy( data->header, bf, sizeof( struct datalinkHeader_ref ) );

			if ( datalinkCheckSumCompute( bf, sizeof( struct datalinkHeader_ref ) - sizeof( int ) * 2 ) == data->header->hcsum &&
					 data->header->messageSize >= sizeof( struct datalinkHeader_ref ) &&
					 data->header->messageSize < BUFFERSIZE )
			{

				if ( data->header->messageSize + index <= port->bytesread )
				{
					/* have read in the entire message */

					/*((struct datalinkHeader_ref *)bf)->hcsum = 0;*/
					if ( datalinkCheckSumCompute( &bf[sizeof( struct datalinkHeader_ref )], data->header->messageSize - sizeof( struct datalinkHeader_ref ) ) == data->header->csum )
					{

						switch ( data->header->messageID )
						{
							case DATALINK_MESSAGE_HEARTBEAT:
							break;

							case DATALINK_MESSAGE_RMTCMD:
							/* the <= thing isn't as stringent as it could be, but this allows backward compatibility */
							/* with older fixed-size version of this packet */
							if ( data->header->messageSize <= sizeof( struct datalinkMessageRmtCmd_ref ) )
							{
								memcpy( data->rmtCmd, bf, data->header->messageSize );
								if ( data->rmtCmd->uniqueID != data->work->lastRmtCmd )
								{

									commandExecute( ( char* ) data->rmtCmd->data );
									data->work->lastRmtCmd = data->rmtCmd->uniqueID;
								}  /* throw away duplicate */
							}
							break;

							case DATALINK_MESSAGE_GETDIR:
							if ( data->header->messageSize == sizeof( struct datalinkMessageGetdir_ref ) )
							{
								int slot, sloti;
								memcpy( data->getdir, bf, sizeof( struct datalinkMessageGetdir_ref ) );

								if ( data->getdir->uniqueID != data->work->getdirID )
								{
									data->work->getdirID = data->getdir->uniqueID;

									if ( data->getdir->dirNum < 0 && data->getdir->updateDt < 0 )
									{ /* clear all downloads */
										for ( sloti = 0; sloti < MAXNUMBEROFSENDDIRS; sloti++ )
										{
											data->set->updateSendDirDt[sloti] = 0;
											data->set->getDirNum[sloti] = -1;
										}
									}
									else
									{
										dataPtr = getDirDataPointer( data->getdir->dirNum, &size );
										if ( size > 0 )
										{ /* fire the data down to the GCS */
											int weWereDownloadingIt = 0;

											if ( data->getdir->updateDt > 0 )
											{ /* continuous update requested */

												for ( sloti = 0; sloti < MAXNUMBEROFSENDDIRS; sloti++ )
												{
													if ( data->set->getDirNum[sloti] == data->getdir->dirNum )
													{
														weWereDownloadingIt = 1;
														/* deal with possible change in update rate */
														data->set->updateSendDirDt[sloti] = data->getdir->updateDt;
													}
												}

												if ( 0 == weWereDownloadingIt )
												{ /* this is a new request */
/* find open slot */
													slot = MAXNUMBEROFSENDDIRS - 1; /* use this one if nothing available */
													for ( sloti = MAXNUMBEROFSENDDIRS - 1; sloti >= 0; sloti-- )
													{
														if ( data->set->getDirNum[sloti] == -1 || data->set->updateSendDirDt[sloti] <= 0 ) slot = sloti;
													}
													/* set up regular download */
													data->set->updateSendDirDt[slot] = data->getdir->updateDt;
													data->set->getDirNum[slot] = data->getdir->dirNum;
												}
											}
											else
											{
												/* turn off all downloads of this */
												for ( sloti = 0; sloti < MAXNUMBEROFSENDDIRS; sloti++ )
												{
													if ( data->set->getDirNum[sloti] == data->getdir->dirNum )
													{
														data->set->updateSendDirDt[sloti] = 0.0;
														data->set->getDirNum[sloti] = -1;
														weWereDownloadingIt = 1;
													}
												}

												if ( 0 == weWereDownloadingIt )
												{
													/* a one-time download */
													memcpy( data->senddir->data, dataPtr, size );
													data->senddir->dirNum = data->getdir->dirNum;
													data->senddir->size = size;
													data->senddir->uniqueID++;
													data->senddir->verbose = 1;

													/* encode message */
													data->senddir->messageID = DATALINK_MESSAGE_SENDDIR;
													data->senddir->messageSize = SENDDIR_BASESIZE + size;

													/* send message */
													datalinkCheckSumEncode( ( unsigned char* ) data->senddir, data->senddir->messageSize );
													writePort( data->p1, ( char* ) data->senddir, data->senddir->messageSize );
													writePort( data->p2, ( char* ) data->senddir, data->senddir->messageSize );
												}
											}
										}
									}
								}
							}
							break;

							case DATALINK_MESSAGE_SENDDIR:
							if ( data->header->messageSize <= SENDDIR_MAXSIZE )
							{
								memcpy( data->senddir, bf, data->header->messageSize );
								dataPtr = getDirDataPointer( data->senddir->dirNum, &size );
								if ( data->senddir->uniqueID != data->work->senddirID )
								{
									data->work->senddirID = data->senddir->uniqueID;
									if ( size == data->senddir->size )
									{
										memcpy( dataPtr, data->senddir->data, size );

										sprintf( gcsPrintBuff, "directory (%d) received onboard", data->senddir->dirNum );
										logInfo( gcsPrintBuff );

										/* send it back for confirmation */
										datalinkCheckSumEncode( ( unsigned char* ) data->senddir, data->senddir->messageSize );
										writePort( data->p1, ( char* ) data->senddir, data->senddir->messageSize );
										writePort( data->p2, ( char* ) data->senddir, data->senddir->messageSize );
									}
									else
									{
										sprintf( gcsPrintBuff, "problem with dir (%d), size was %d, compared to %d", data->senddir->dirNum, size, data->senddir->size );
										logWarning( gcsPrintBuff );
									}
								}
							}
							break;

							case DATALINK_MESSAGE_UP0:
							if ( data->header->messageSize == sizeof( struct datalinkMessageUp0_ref ) )
							{
								if ( !( ( ( UPIGNORE_P1 == data->set->ignoreUpMask ) && ( port == data->p1 ) ) ||
												( ( UPIGNORE_P2 == data->set->ignoreUpMask ) && ( port == data->p1 ) ) ||
												( ( UPIGNORE_P1P2 == data->set->ignoreUpMask ) && ( ( port == data->p1 ) || ( port == data->p2 ) ) ) ||
												( ( UPIGNORE_ALL == data->set->ignoreUpMask ) && ( ( port == data->p1 ) || ( port == data->p2 ) ) )
												) ) { //Don't process this message if any of the above are true (port is to be ignored and it is that given port(s))

									memcpy( data->up0, bf, sizeof( struct datalinkMessageUp0_ref ) );

									if ( ISFINITE( data->up0->throttleLever ) &&
											 ISFINITE( data->up0->rollStick ) &&
											 ISFINITE( data->up0->pitchStick ) &&
											 ISFINITE( data->up0->rudderPedal ) )
									{
										data->m1->yrdStatus = 1; // good receiver data

										// save RC commands and add to message1 to send back to gcs
										ob->actuators->work->delm[0] = LIMIT( data->up0->rollStick, -1.0, 1.0 );
										ob->actuators->work->delm[1] = LIMIT( data->up0->pitchStick, -1.0, 1.0 );
										ob->actuators->work->delm[2] = LIMIT( data->up0->rudderPedal, -1.0, 1.0 );
										ob->actuators->work->delf[0] = LIMIT( data->up0->throttleLever, -1.0, 1.0 );
									}
								}
							}
							break;

							default:
							/* unrecognized message */
							break;
						}

						data->work->itime++;
						*rxTime = nav->out->time;
					}
					else
					{ /* checksum bad */
						data->work->badChecksums++;
					}
					index += data->header->messageSize - 1;

				}
				else
				{ /* end of buffer includes a partial message - come back later... */
					index--;
					done = 1;
				}
			}
			else
			{ /* header checksum is bad */
				index += sizeof( struct datalinkHeader_ref ) - 1;
				data->work->badHeaderChecksums++;
			}
		}
		index++; /* start seq not found, go to next byte */

		if ( index < 0 ) index = BUFFERSIZE - 1;
	}
	clearPort( port, index );
}

static void sendM0( struct onboard_ref* ob, struct serialPort_ref* port )
{
	struct obDatalink_ref* data = ob->datalink;
	struct sensors_ref* sen = ob->sensors;
	struct navigation_ref* nav = ob->navigation;
	struct onboardControl_ref* con = ob->control;

	int i;

	data->m0->time = ( float ) nav->out->time;

	for ( i = 0; i < 3; i++ )
	{
		data->m0->pos[i] = ( float ) nav->out->p_b_e_L[i];
		data->m0->vel[i] = ( float ) nav->out->v_b_e_L[i];
	}

	for ( i = 0; i < 4; i++ )
	{
		data->m0->q[i] = ( float ) nav->out->q[i];
	}

	data->m0->altitudeAGL = ( float ) nav->out->altitudeAGL;
	data->m0->wow = nav->out->wow;

	// Handle special augmented/rate feedback safety pilot radio with autopilot modes

	/* gps status */
	if ( nav->out->time > sen->gps->out->lastUpdate + sen->gps->set->timeOut ) sen->gps->out->gpsStatus = GPS_OFF;
	if ( nav->out->time < sen->gps->out->lastUpdate ) sen->gps->out->lastUpdate = -99.0;
	if ( nav->out->time > sen->gps->out->lastUpdateRoverposb + sen->gps->set->timeOut ) sen->gps->out->alignAvailable = 0;
	if ( nav->out->time < sen->gps->out->lastUpdateRoverposb ) sen->gps->out->lastUpdateRoverposb = -99.0;
	if ( sen->gps->out->gpsStatus == GPS_SINGLE && sen->gps->out->alignAvailable ) sen->gps->out->gpsStatus = GPS_SINGLE_ALIGN_AVAILABLE;
	data->m0->gpsStatus = sen->gps->out->gpsStatus;

	/* other status flags */
	data->m0->navStatus = nav->work->navStatus;
	if ( nav->work->timeRotorStartup <= nav->set->rotorSpinTime )
	{
		data->m0->motor = ( unsigned char ) LIMIT( nav->work->timeRotorStartup + 1.99, 1, 255 ); // 0 and 1 on this flag have different meanings
	}
	else
	{
		data->m0->motor = 0;
	}

	// AGL sensor update
	unsigned char aglStatus = ( sen->agl->out->status == AGL_OUTLIER_CONSISTENT ) ? AGL_OUTLIER_DATALINK : sen->agl->out->status;
	data->m0->aglStatus = aglStatus | ( sen->agl->out->source << 2 ); // Encode the status (bits 0-1) and the source (bits 2-4)
	/* encode message */
	data->m0->messageID = DATALINK_MESSAGE0;
	data->m0->messageSize = sizeof( struct datalinkMessage0_ref );

	/* send message */
	datalinkCheckSumEncode( ( unsigned char* ) data->m0, sizeof( struct datalinkMessage0_ref ) );
	writePort( port, ( char* ) data->m0, sizeof( struct datalinkMessage0_ref ) );
}

static void sendM1( struct onboard_ref* ob )
{
	struct obDatalink_ref* data = ob->datalink;
	struct sensors_ref* sen = ob->sensors;
	struct navigation_ref* nav = ob->navigation;
	struct onboardControl_ref* con = ob->control;

	int i;

	data->m1->time = ( float ) nav->out->time;
	data->m1->type = ob->set->type;
	data->m1->uniqueID = ob->set->uniqueID;

	for ( i = 0; i < 3; i++ )
	{
		data->m1->delm[i] = ( float ) ob->actuators->work->delm[i];
		data->m1->wind[i] = ( float ) ob->navigation->out->v_a_e_L[i];
	}
	data->m1->delf[0] = ( float ) ob->actuators->work->delf[0];
	data->m1->delt[0] = ( float ) ob->actuators->work->delt[0];
	data->m1->delc[0] = ( float ) ob->actuators->work->delc[0];

	data->m1->datumLat = ( float ) ob->navigation->navinit->datumLat;
	data->m1->datumLon = ( float ) ob->navigation->navinit->datumLon;
	data->m1->datumAlt = ( float ) ob->navigation->navinit->datumAlt;

	data->m1->G = ( float ) ( sqrt(
		SQ( nav->work->s_b_e_B[0] )
		+ SQ( nav->work->s_b_e_B[1] )
		+ SQ( nav->work->s_b_e_B[2] ) ) / 32.174 );

	// Update GPS data.  This is changed to reflect the selection of the GPS sensor, not the open port
	// All common data
	data->m1->numberOfSats = ob->sensors->gps->out->numberOfObsTr;
	data->m1->ubloxSNR = ( unsigned char ) ob->sensors->gps->out->averageSnr;
	data->m1->ubloxHacc = ( unsigned char ) ( MIN( ob->sensors->gps->out->hacc / 100, 255 ) );
	data->m1->ubloxSacc = ( unsigned char ) ( MIN( ob->sensors->gps->out->sacc / 10, 255 ) );
	data->m1->ubloxPDOP = ( unsigned char ) ( MIN( ob->sensors->gps->out->pdop / 10, 255 ) );

	data->m1->rpm = ( int ) ( nav->out->rpm );
	data->m1->fuel = 0;
	if ( nav->out->time > sen->magnet->out->lastUpdate + sen->magnet->set->timeOut ) sen->magnet->out->magnetStatus = MAGNET_OFF;
	if ( nav->out->time < sen->magnet->out->lastUpdate ) sen->magnet->out->lastUpdate = -99.0;
	data->m1->magnetStatus = sen->magnet->out->magnetStatus;

	// Set m1->rangeFinderStatus.  This field has the following meanings by bit:
	// (0,1) = number of AGL sensors (0-3)
	// (2,3) = status of the second AGL sensor (AGL_OFF, AGL_OUTOFRANGE, AGL_GOOD, AGL_OUTLIER_DATALINK)
	// (4,5) = status of the third AGL sensor (AGL_OFF, AGL_OUTOFRANGE, AGL_GOOD, AGL_OUTLIER_DATALINK)
	// (6,7) = status of a range finder not being used as an AGL sensor (off if there isn't one) (AGL_OFF, AGL_OUTOFRANGE, AGL_GOOD)
	struct senAGL_ref* agl = sen->agl;
	unsigned char numAglSensors = 0;
	unsigned char extraAglSensor = 0;
	unsigned char aglSensor2Status = 0;
	unsigned char aglSensor3Status = 0;
	unsigned char otherRangeSensorStatus = 0;
	for ( unsigned char aglSensorIdx = 0; aglSensorIdx < MAX_AGL_SOURCES; ++aglSensorIdx )
	{
		if ( agl->set->sources[aglSensorIdx] != AGL_SOURCE_NONE )
		{
			// Is a source
			numAglSensors += 1;
			struct aglOut_ref* tempAgl = agl->temp0;
			switch ( numAglSensors )
			{
				case 2:
				tempAgl = agl->temp1;
				break;
				case 3:
				tempAgl = agl->temp2;
			}
			if ( tempAgl->source != agl->out->source )
			{
				// Not the currently selected one - add its status
				switch ( extraAglSensor )
				{
					case 0:
					aglSensor2Status = ( tempAgl->status == AGL_OUTLIER_CONSISTENT ) ? AGL_OUTLIER_DATALINK : tempAgl->status;
					break;
					case 1:
					aglSensor3Status = ( tempAgl->status == AGL_OUTLIER_CONSISTENT ) ? AGL_OUTLIER_DATALINK : tempAgl->status;
				}
				++extraAglSensor; // Index for next round
			}
		}
	}

	if ( sim.time > sen->imu->out->lastUpdateTime + sen->imu->timeOut ) sen->imu->out->status = 0;
	data->m1->imuStatus = sen->imu->out->status;

	data->m1->actuatorInterfaceStatus = 0; // Armed with active SAS

	if ( data->work->rxTimeLink1 + data->set->uplinkTimeOut > nav->out->time )
	{
		data->m1->uplinkStatus[0] = 1;
	}
	else
	{
		data->m1->uplinkStatus[0] = 0;
		if ( data->p1->dataSource == PORT_ON && data->set->resetIfDown ) data->p1->init = 1;
		data->work->rxTimeLink1 = MIN( data->work->rxTimeLink1, nav->out->time - data->set->uplinkTimeOut );  /* in case nav->out->time starts over */
	}
	if ( data->work->rxTimeLink2 + data->set->uplinkTimeOut > nav->out->time )
	{
		data->m1->uplinkStatus[1] = 1;
	}
	else
	{
		data->m1->uplinkStatus[1] = 0;
		if ( data->p2->dataSource == PORT_ON && data->set->resetIfDown ) data->p2->init = 1;
		data->work->rxTimeLink2 = MIN( data->work->rxTimeLink2, nav->out->time - data->set->uplinkTimeOut );  /* in case nav->out->time starts over */
	}
	//data->m1->lostComm = ob->trajectory->lostComm->triggered;

	/* encode message */
	data->m1->messageID = DATALINK_MESSAGE1;
	data->m1->messageSize = sizeof( struct datalinkMessage1_ref );

	/* send message */
	datalinkCheckSumEncode( ( unsigned char* ) data->m1, sizeof( struct datalinkMessage1_ref ) );
	writePort( data->p1, ( char* ) data->m1, sizeof( struct datalinkMessage1_ref ) );
	writePort( data->p2, ( char* ) data->m1, sizeof( struct datalinkMessage1_ref ) );

	data->m1->yrdStatus = 0; // reset receiver flag
}

void updateDatalink( struct onboard_ref* ob )
{
	struct obDatalink_ref* data = ob->datalink;
	struct sensors_ref* sen = ob->sensors;
	struct navigation_ref* nav = ob->navigation;
	struct onboardControl_ref* con = ob->control;

	double superTime;

	int i;

	if ( ob->init )
	{
		data->work->lastUpdate0_1 = -data->set->updateDt0_1;
		data->work->lastUpdate0_2 = -data->set->updateDt0_2;
		data->work->lastUpdate1 = -data->set->updateDt1;
		data->work->lastUpdate1B = -data->set->updateDt1 * 0.5; /* so it will out of phase with 1 */
		data->work->lastUpdate2 = -data->set->updateDt2 * 0.25; /* out of phase with 1 and 1b */
		for ( i = 0; i < MAXNUMBEROFSENDDIRS; i++ ) data->work->lastUpdateSendDir[i] = -1.0;
		data->work->lastUpdateIPC0 = -data->set->updateDtIPC0;
		data->work->lastUpdateVFIPC = -data->set->updateDtVFIPC;
		data->work->lastUpdateExtMan0 = -data->set->updateDtExtMan0;
		data->work->lastUpdateExtMan2 = -data->set->updateDtExtMan2;
		data->work->lastUpdateCam0 = -data->set->updateDtCam0;
		data->work->lastUpdateSquitter = -data->set->updateDtSquitter;
		data->work->lastUpdateSquitterMini = -data->set->updateDtSquitterMini;
		data->work->lastUpdateYJData = -data->set->updateDtYJData;
		data->work->lastUpdateDownloadingPlan = -data->set->updateDtDownloadingPlan;
		data->work->lastUpdateHealth = -data->set->updateDtHealth;
		data->work->lastUpdateGpsDetails = -data->set->updateDtGpsDetails;
		data->work->lastUpdateAutopilotDels = -data->set->updateDtAutopilotDels;
		data->work->lastUpdateNavTime = -data->set->updateDtNavTime;
		data->work->lastUpdateExtMan3 = -data->set->updateDtExtMan3;
	}

	/* now send messages as needed */

	superTime = MAX( sim.time - sen->imu->timeOut, nav->out->time );

	if ( superTime >= data->work->lastUpdate0_1 + data->set->updateDt0_1 )
	{

		sendM0( ob, data->p1 );

		/* deal with counters */
		data->work->lastUpdate0_1 =
			MAX( data->work->lastUpdate0_1 + data->set->updateDt0_1, superTime - data->set->updateDt0_1 );
	}

	if ( superTime >= data->work->lastUpdate0_2 + data->set->updateDt0_2 )
	{
		sendM0( ob, data->p2 );

		/* deal with counters */
		data->work->lastUpdate0_2 =
			MAX( data->work->lastUpdate0_2 + data->set->updateDt0_2, superTime - data->set->updateDt0_2 );
	}

	if ( superTime >= data->work->lastUpdate1 + data->set->updateDt1 )
	{
		sendM1( ob );

		/* deal with counters */

		data->work->lastUpdate1 =
			MAX( data->work->lastUpdate1 + data->set->updateDt1, superTime - data->set->updateDt1 );
	}

	for ( i = 0; i < MAXNUMBEROFSENDDIRS; i++ )
	{
		if ( data->set->updateSendDirDt[i] > 0.0 )
		{
			if ( nav->out->time >= data->work->lastUpdateSendDir[i] + data->set->updateSendDirDt[i] ||
					 nav->out->time < data->work->lastUpdateSendDir[i] )
			{

				void* dataPtr;
				int size;

				dataPtr = getDirDataPointer( data->set->getDirNum[i], &size );
				if ( size > 0 )
				{ /* fire the data down to the GCS */
					memcpy( data->senddir->data, dataPtr, size );
					data->senddir->dirNum = data->set->getDirNum[i];
					data->senddir->size = size;
					data->senddir->uniqueID++;
					data->senddir->verbose = 0;

					/* encode message */
					data->senddir->messageID = DATALINK_MESSAGE_SENDDIR;
					data->senddir->messageSize = SENDDIR_BASESIZE + size;

					/* send message */
					datalinkCheckSumEncode( ( unsigned char* ) data->senddir, data->senddir->messageSize );
					writePort( data->p1, ( char* ) data->senddir, data->senddir->messageSize );
					writePort( data->p2, ( char* ) data->senddir, data->senddir->messageSize );
				}

				data->work->lastUpdateSendDir[i] =
					MAX( data->work->lastUpdateSendDir[i] + data->set->updateSendDirDt[i],
							 nav->out->time - data->set->updateSendDirDt[i] );
			}
		}
	}

	if ( ( 2 == data->set->sendAutopilotDels ) ||
			 ( 1 == data->set->sendAutopilotDels /* && 0 == ob->sensors->ycs->out->autopilot*/ ) )
	{

		if ( superTime > data->set->updateDtAutopilotDels + data->work->lastUpdateAutopilotDels )
		{

			data->autopilotDels->time = ( float ) nav->out->time;
			for ( i = 0; i < 3; i++ ) data->autopilotDels->c_delm[i] = ( float ) ( ob->actuators->work->c_delm[i] );
			data->autopilotDels->c_delf[0] = ( float ) ( ob->actuators->work->c_delf[0] );
			data->autopilotDels->c_delt[0] = ( float ) ( ob->actuators->work->c_delt[0] );

			/* encode message */
			data->autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
			data->autopilotDels->messageSize = sizeof( struct datalinkMessageAutopilotDels_ref );

			/* send message */
			datalinkCheckSumEncode( ( unsigned char* ) data->autopilotDels, data->autopilotDels->messageSize );
			writePort( data->p1, ( char* ) data->autopilotDels, data->autopilotDels->messageSize );
			writePort( data->p2, ( char* ) data->autopilotDels, data->autopilotDels->messageSize );

			data->work->lastUpdateAutopilotDels = superTime;
		}
	}

	/* datalink reads */
	readDatalink( ob, data->p1, &( data->work->rxTimeLink1 ) );
	readDatalink( ob, data->p2, &( data->work->rxTimeLink2 ) );
}

//downlink output from the logger

//data->set->rmtCnsl:
//0. never send
//1. (or any other positive number)  send (no rate limit)
//2. don't sent if log message occur faster than a specified rate
//3. allow send with unlimited rate only if the message is different than the previous one
void datalinkLog( const char* out )
{
	struct obDatalink_ref* data = &obDatalink;
	static char prevMsg[RMTCNSL_MAXSIZE];

	if ( data->set->rmtCnsl > 0 && onboard.run )
	{

		if ( data->set->rmtCnsl == 3 )
		{
			memcpy( prevMsg, data->rmtCnsl->data, data->rmtCnsl->size );
		}

		data->rmtCnsl->size = MIN( strlen( out ) + 1, RMTCNSL_MAXSIZE ); // plus 1 to get the null
		memcpy( data->rmtCnsl->data, out, data->rmtCnsl->size );
		data->rmtCnsl->data[data->rmtCnsl->size - 1] = '\0'; // make sure it is null terminated 

		if ( sim.time < data->work->lastUpdateRmtCnsl )
		{ //in case there is a reset
			data->work->lastUpdateRmtCnsl = sim.time;
		}

		if ( sim.time - data->work->lastUpdateRmtCnsl < data->set->minDtRmtCnsl && data->set->minDtRmtCnsl > 0 )
		{

			data->work->logOverrun++;

			if ( data->set->rmtCnsl == 2 )
			{ //skip the send if faster than the specified rate
				return;

				//skip the send if faster than the specified rate and the message is the same
				//(so if the messages are different, they will still get sent at an unrestricted rate)
			}
			else if ( data->set->rmtCnsl == 3 && ( 0 == memcmp( prevMsg, data->rmtCnsl->data, data->rmtCnsl->size ) ) )
			{
				return;
			}
		}

		data->rmtCnsl->uniqueID++;
		data->rmtCnsl->nOverruns = data->work->logOverrun;
		data->rmtCnsl->messageID = DATALINK_MESSAGE_RMTCNSL;
		data->rmtCnsl->messageSize = sizeof( struct datalinkMessageRmtCnsl_ref ) - RMTCNSL_MAXSIZE + data->rmtCnsl->size;
		datalinkCheckSumEncode( ( unsigned char* ) data->rmtCnsl, data->rmtCnsl->messageSize );
		writePort( data->p1, ( char* ) data->rmtCnsl, data->rmtCnsl->messageSize );
		writePort( data->p2, ( char* ) data->rmtCnsl, data->rmtCnsl->messageSize );
		data->work->lastUpdateRmtCnsl = sim.time;
		data->work->logOverrun = 0;
	}
}

void datalinkCnslPrint( const char* out )
{
	logEvent( "cnsl.c", "cnslAddLine", LOG_LEVEL_INFO, 0, out );
}

int datalink_close( struct onboard_ref* ob )
{
	closePort( ob->datalink->p1 );
	closePort( ob->datalink->p2 );
	return 0;
}
