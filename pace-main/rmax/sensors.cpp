#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esim/util.h"
#include "esim/quat.h"
#include "esim/cnsl.h"
#include "esim/sim_ref.h"
#include "rmax/serial.h"
#include "rmax/sensors_ref.h"
#include "rmax/sensors.h"
#include "rmax/checksum.h"
#include "rmax/onboard_ref.h"
#include "rmax/controller_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/matrix.h"
#include "rmax/aglSensor.h"
#include "rmax/novatel.h"
#include "rmax/datalink.h"
#include "rmax/logger.h"
#include "rmax/wdb_ref.h"
#include "rmax/wdb.h"

static unsigned char imuCheckSumCompute ( unsigned char* buf, int byteCount )
{

	int m;
	unsigned char csum;

	csum = 0;
	for ( m = 0; m < byteCount; m++ )
		csum ^= buf[m];

	return( csum );

}

static unsigned char ycsCheckSumCompute ( unsigned char* buf, int byteCount )
{

	int m;
	unsigned char csum;

	csum = 0;
	for ( m = 0; m < byteCount; m++ )
		csum ^= buf[m];

	return( csum );

}


static unsigned short decodeYRD ( unsigned char index, unsigned char* bf )
{

	if ( index == ( ( bf[0] & 0x7c ) >> 2 ) )
		return( ( ( bf[0] & 0x03 ) << 7 ) + ( bf[1] & 0x7f ) ) * 1200 / 512 + 900;
	else
		return 1500;

}

static short decodeYAS ( unsigned char index, unsigned char* bf )
{

	if ( index == ( ( bf[0] & 0x70 ) >> 4 ) )
		if ( bf[0] & 0x08 )
			return +( ( ( bf[0] & 0x07 ) << 7 ) + ( bf[1] & 0x7f ) );
		else
			return -( ( ( bf[0] & 0x07 ) << 7 ) + ( bf[1] & 0x7f ) );
	else
		return 0;

}

/* this will take a 16-bit value and break it into 2 bytes */
void aveoxBreak16Bit ( unsigned short input, unsigned char* high, unsigned char* low )
{
	*high = ( unsigned char ) ( ( input & 0xFF00 ) >> 8 );
	*low = ( unsigned char ) ( input & 0x00FF );
}

void mapControlsPWM ( struct pwmServoMapping_ref* map,
											struct pwmServoMixing_ref* mix,
											struct pwmMTRServoMixing_ref* mmix,
											struct pwmServoMapping_ref* inmap,
											struct datalinkMessagePWM_ref* pwm,
											double* roll, double* pitch, double* yaw,
											double* thrust, double* throttle, double* nws, double* flaps,
											double* autopilot,
											short* spMode, short allowSPOverride )
{

	switch ( map->mixMode )
	{
		case MIX_NONE:
		case MIX_MULTIROTOR:  /* treat safety pilot data as "normal" 4 channel input */
		default:
		*autopilot = ( double ) ( pwm->channel[map->autopilot] - map->bias[map->autopilot] ) / map->gain[map->autopilot];
		*roll = ( double ) ( pwm->channel[map->roll] - map->bias[map->roll] ) / map->gain[map->roll] * map->rev[map->roll];
		*pitch = ( double ) ( pwm->channel[map->pitch] - map->bias[map->pitch] ) / map->gain[map->pitch] * map->rev[map->pitch];
		*yaw = ( double ) ( pwm->channel[map->yaw] - map->bias[map->yaw] ) / map->gain[map->yaw] * map->rev[map->yaw];
		*thrust = ( double ) ( pwm->channel[map->thrust] - map->bias[map->thrust] ) / map->gain[map->thrust] * map->rev[map->thrust];
		if ( map->gain[map->throttle] != 0 )
			*throttle = ( double ) ( pwm->channel[map->throttle] - map->bias[map->throttle] ) / map->gain[map->throttle];
		else
			*throttle = 0.0;
		if ( map->gain[map->nws] != 0 )
			*nws = ( double ) ( pwm->channel[map->nws] - map->bias[map->nws] ) / map->gain[map->nws];
		else
			*nws = 0.0;
		if ( map->gain[map->flaps] != 0 )
			*flaps = ( double ) ( pwm->channel[map->flaps] - map->bias[map->flaps] ) / map->gain[map->flaps];
		else
			*flaps = 0.0;
		if ( map->mixMode == MIX_ELEVON )
		{ /* replace for elevon case */
			double servo0, servo1;
			servo0 = ( double ) ( pwm->channel[map->yaw] - map->bias[map->yaw] ) * map->rev[0];
			servo1 = ( double ) ( pwm->channel[map->pitch] - map->bias[map->pitch] ) * map->rev[1];
			*yaw = ( servo0 - servo1 ) * 0.5 / map->gain[map->yaw];
			*pitch = ( servo0 + servo1 ) * 0.5 / map->gain[map->pitch];
		}
		break;

		case 99:
		break;
	} /* end switch on mixing mode */

}

/* This closes all sensors */
int sensors_close ( struct onboard_ref* ob )
{
	closePort ( ob->sensors->imu->p );
	closePort ( ob->sensors->gps->novatel1->p );
	closePort ( ob->sensors->magnet->p );
	closePort ( ob->sensors->agl->sonar0->p );

	return 0;
}

void readIMU ( struct onboard_ref* ob )
{

	struct sensors_ref* sen = ob->sensors;
	struct senImu_ref* imu = sen->imu;
	struct serialPort_ref* port;
	struct actuatorInt_ref* act = ob->actuators;

	int index, done;
	unsigned char* bf;

	/* read new IMU data */

#define IMUSIZE 16

	port = imu->p;

	readPort ( port );
	if ( port->dataSource != PORT_OFF )
	{

		done = 0;
		index = 0;

		imu->out->newIMUData = 0;

		if ( imu->input == 0 )
		{
			/* look for a header */
			while ( index <= port->bytesread - IMUSIZE && !done )
			{

				if ( port->buffer[index] == 0x7E )
				{   /* sequence header */

					bf = &( port->buffer[index] );

					imu->ourcsum = imuCheckSumCompute ( bf, IMUSIZE - 1 );

					/* read the message segment if csum is correct */
					if ( bf[15] == imu->ourcsum )
					{

						double w_b_e_S[3], s_b_e_S[3];
						int j;

						memcpy ( imu->raw, bf, sizeof ( struct imuRaw_ref ) );

						imu->out->itime++;
						imu->out->status = 1;
						imu->out->lastUpdateTime = sim.time;

						for ( j = 0; j < 3; j++ )
						{
							w_b_e_S[j] = imu->raw->rate[j] * imu->sfr;
							s_b_e_S[j] = imu->raw->accel[j] * imu->sfa;
						}

						map_vector ( imu->dcm_sb, w_b_e_S, imu->out->w_b_e_B );
						map_vector ( imu->dcm_sb, s_b_e_S, imu->out->s_b_e_B );

						/*imu->out->w_b_e_B[0] = +imu->raw->rate[0] *imu->sfr; upside down
						imu->out->w_b_e_B[1] = -imu->raw->rate[1] *imu->sfr;
						imu->out->w_b_e_B[2] = -imu->raw->rate[2] *imu->sfr;
						imu->out->s_b_e_B[0] = +imu->raw->accel[0]*imu->sfa;
						imu->out->s_b_e_B[1] = -imu->raw->accel[1]*imu->sfa;
						imu->out->s_b_e_B[2] = -imu->raw->accel[2]*imu->sfa;*/

						imu->out->count = imu->raw->count;

						/* logic to process packets multiple times if some dropped */
						if ( imu->out->count >= 0 && imu->out->count < 100 )
						{
							imu->out->checkCount++; /* determine what we think counter should be for new packet */
							imu->out->checkCount %= 100;
							while ( imu->out->count != imu->out->checkCount )
							{
								imu->out->checkCount++;
								imu->out->checkCount %= 100;
							}
						}
					}
					imu->out->newIMUData = 1;
					done = 1; /* this line is important, so nav will take place on each packet */

					index += IMUSIZE - 1;
				}
				index++; /* start seq not found, go to next byte */

				if ( index < 0 ) index = BUFFERSIZE - 1;
			}
		}
		else
		{ // Swap space data - all floats with mag data (potentially) included
/* look for a header */
			while ( index <= port->bytesread - ( int ) sizeof ( struct imuFloatRaw_ref ) && !done )
			{

				if ( ( port->buffer[index] == IMU_SYNC0 ) &&
						 ( port->buffer[index + 1] == IMU_SYNC1 ) &&
						 ( port->buffer[index + 2] == IMU_SYNC2 ) )
				{   /* sequence header */
// Found the header
					bf = &( port->buffer[index] );

					double w_b_e_S[3], s_b_e_S[3];
					int j;

					memcpy ( imu->floatRaw, bf, sizeof ( struct imuFloatRaw_ref ) );

					imu->out->itime++;
					imu->out->count++;
					imu->out->status = 1;
					imu->out->lastUpdateTime = sim.time;

					for ( j = 0; j < 3; j++ )
					{
						w_b_e_S[j] = ( double ) imu->floatRaw->rate[j] * imu->sfr; // Scaling available
						s_b_e_S[j] = ( double ) imu->floatRaw->accel[j] * imu->sfa; // Scaling available
					}

					map_vector ( imu->dcm_sb, w_b_e_S, imu->out->w_b_e_B );
					map_vector ( imu->dcm_sb, s_b_e_S, imu->out->s_b_e_B );

					// Check if magnetometer data is included
					if ( imu->floatRaw->hasMag == 1 )
					{
						// The magnetometer data needs to be stuffed into the magnetometer system raw data and notified
						for ( int index = 0; index < 3; ++index )
						{
							sen->magnet->rawFloat->values[index] = imu->floatRaw->mag[index];
						}
						sen->magnet->rawFloat->newValue++;
					}
					imu->out->newIMUData = 1;
					done = 1; /* this line is important, so nav will take place on each packet */

					index += sizeof ( struct imuFloatRaw_ref ) - 1;
				}
				index++; /* start seq not found, go to next byte */

				if ( index < 0 ) index = BUFFERSIZE - 1;
			}
		}

		clearPort ( port, index );
	}

}

int magnetHMR3400Update ( struct onboard_ref* ob )
{

	struct senMagnet_ref* magnet = ob->sensors->magnet;
	struct serialPort_ref* port = magnet->p;

	int index = 0;
	int done = 0;

	while ( index <= port->bytesread && !done )
	{
		if ( ( port->buffer[index] == 0x0D ) && ( port->buffer[index + 1] == 0x0A ) )
		{
			/* found the end of the magnetometer message */
			done = 1;

			/* message is "100,200,300,C<CR><LF>" in calibration mode */
			char* calibrate = ( char* ) memchr ( port->buffer, 'C', index );

			if ( calibrate != NULL )
			{
				magnet->out->calibMode = 1;
			}
			else
			{
				magnet->out->calibMode = 0;
			}

			port->buffer[index] = '\0';
			port->buffer[index + 1] = '\0';

			char* magX = strtok ( ( char* ) port->buffer, ", " );
			char* magY = strtok ( NULL, ", " );
			char* magZ = strtok ( NULL, ", " );

			/* for a valid message all of these will have a value */
			if ( ( magX != NULL ) && ( magY != NULL ) && ( magZ != NULL ) )
			{
				magnet->raw->values[0] = atoi ( magX );
				magnet->raw->values[1] = atoi ( magY );
				magnet->raw->values[2] = atoi ( magZ );
			}
		}

		/* end sequence is not found, so go to the next byte */
		index++;
	}

	return index;

}

void magnetUpdate ( struct onboard_ref* ob )
{

	struct senMagnet_ref* magnet = ob->sensors->magnet;
	struct serialPort_ref* port = magnet->p;

	if ( magnet->set->magnetType == MAGTYPE_RAWFLOAT )
	{
		if ( magnet->rawFloat->newValue > 0 )
		{
			double mag_s[3] = { 0.0, 0.0, 0.0 };
			for ( int index = 0; index < 3; ++index )
			{
				mag_s[index] = ( double ) magnet->rawFloat->values[index]; // Bias handled elsewhere
			}

			/* transform measurements to the body frame */
			double mag_b[3] = { 0.0, 0.0, 0.0 };
			map_vector ( magnet->set->dcm_sb, mag_s, mag_b );

			/* normalize */
			double vecLength = sqrt ( ( SQ ( mag_b[0] ) + SQ ( mag_b[1] ) + SQ ( mag_b[2] ) ) );

			if ( vecLength > magnet->set->minStrength )
			{
				magnet->out->itime++;
				magnet->out->lastUpdate = ob->navigation->out->time;

				// No calibration mode for this magnetometer
				magnet->out->magnetStatus = MAGNET_ON;

				magnet->out->field_B[0] = mag_b[0] / vecLength;
				magnet->out->field_B[1] = mag_b[1] / vecLength;
				magnet->out->field_B[2] = mag_b[2] / vecLength;

				magnet->out->rawFloat[0] = ( double ) magnet->rawFloat->values[0]; // For data recording purposes
				magnet->out->rawFloat[1] = ( double ) magnet->rawFloat->values[1]; // For data recording purposes
				magnet->out->rawFloat[2] = ( double ) magnet->rawFloat->values[2]; // For data recording purposes
			}

			// Reset flag for next data
			magnet->rawFloat->newValue = 0;
		} // else: no new data - do nothing
	}
	else
	{
		readPort ( port );
		if ( port->dataSource == PORT_OFF ) return;

		int bytes = 0;

		switch ( magnet->set->magnetType )
		{
			case MAGTYPE_HMR3400:
			bytes = magnetHMR3400Update ( ob );
			break;
			case MAGTYPE_RMAX:
			default:
			break;
		}

		clearPort ( port, bytes );

		double mag_s[3] = { 0.0, 0.0, 0.0 };
		mag_s[0] = ( double ) ( magnet->raw->values[0] - magnet->set->bias[0] );
		mag_s[1] = ( double ) ( magnet->raw->values[1] - magnet->set->bias[1] );
		mag_s[2] = ( double ) ( magnet->raw->values[2] - magnet->set->bias[2] );

		/* transform measurements to the body frame */
		double mag_b[3] = { 0.0, 0.0, 0.0 };
		map_vector ( magnet->set->dcm_sb, mag_s, mag_b );

		/* normalize */
		double vecLength = sqrt ( ( SQ ( mag_b[0] ) + SQ ( mag_b[1] ) + SQ ( mag_b[2] ) ) );

		if ( vecLength > magnet->set->minStrength )
		{
			magnet->out->itime++;
			magnet->out->lastUpdate = ob->navigation->out->time;

			if ( magnet->out->calibMode == 1 )
			{
				magnet->out->magnetStatus = MAGNET_CALIBRATE;
			}
			else
			{
				magnet->out->magnetStatus = MAGNET_ON;
			}

			magnet->out->field_B[0] = mag_b[0] / vecLength;
			magnet->out->field_B[1] = mag_b[1] / vecLength;
			magnet->out->field_B[2] = mag_b[2] / vecLength;
		}
	}
}

void sensors_update ( struct onboard_ref* ob )
{

	struct sensors_ref* sen = ob->sensors;
	struct navinit_ref* init = ob->navigation->navinit;
	struct senAGL_ref* agl = sen->agl;
	struct senGps_ref* gps = sen->gps;
	struct senMagnet_ref* magnet = sen->magnet;
	struct actuatorInt_ref* act = ob->actuators;
	struct serialPort_ref* port;

	struct obDatalink_ref* data = ob->datalink;

	int index, j, done; //, notSynched;
	unsigned char* bf;

	/* imu read has been moved to readIMU func called directly from onboard.cpp */

	/* read new magnetometer data */

#define MAGSIZE 7
	if ( magnet->set->enableTestCode == 1 )
	{

		magnetUpdate ( ob );

	}
	else
	{

		port = magnet->p;

		readPort ( port );
		if ( port->dataSource != PORT_OFF )
		{

			done = 0;
			index = 0;

			if ( magnet->set->magnetType == MAGTYPE_HMR3400 )
			{

				int foundFirstMagMarker = 0;
				int magMsgEnd = 0;
				int magMsgStart;
				char magMsgXstring[10];
				char magMsgYstring[10];
				char magMsgZstring[10];
				int magCommaIdx = 0;
				int magSeparator[2];

				/* send generic text to device */
				if ( magnet->set->config )
				{
					magnet->set->config = 0;
					char buffer[40];
					sprintf ( buffer, "%s%c%c", magnet->set->configText, 0x0D, 0x0A );
					writePort ( port, buffer, strlen ( buffer ) );
				}

				//i = MAX( 0, port->bytesread - 33 ); /* 33 bytes is the biggest it can be - just get the newest one please */

				/* loop through until we find the first magnetometer message marker.
					 the 9 represets the shortest complete message we can process of <cr><lf>0,0,0<cr><lf> */
				while ( ( index <= port->bytesread - 9 ) && !done )
				{
					if ( ( port->buffer[index] == 0x0D ) && ( port->buffer[index + 1] == 0x0A ) )
					{
						/* mark where we found the magnetometer msg marker */
						magMsgEnd = index;
						foundFirstMagMarker = 1;
						index = index + 2;
						done = 1;
					}
					else
					{
						index++;
					}
					if ( index < 0 ) index = BUFFERSIZE - 1;
				} /* end loop to read first marker for the HMR3400 message */

				done = 0;
				while ( ( index <= port->bytesread - 2 ) && !done && foundFirstMagMarker )
				{
					if ( ( port->buffer[index] == 0x0D ) && ( port->buffer[index + 1] == 0x0A ) )
					{
						/* process the last string of data */
						magMsgStart = magMsgEnd + 2;
						magMsgEnd = index;

						/* check that the message is valid */
						if ( magCommaIdx == 2 )
						{
							/* number of bytes for each string */
							int magMsgXstringSize = MIN ( magSeparator[0] - magMsgStart, 9 );
							int magMsgYstringSize = MIN ( magSeparator[1] - ( magSeparator[0] + 1 ), 9 );
							int magMsgZstringSize = MIN ( magMsgEnd - ( magSeparator[1] + 1 ), 9 );

							memcpy ( magMsgXstring, &( port->buffer[magMsgStart] ), magMsgXstringSize );
							memcpy ( magMsgYstring, &( port->buffer[magSeparator[0] + 1] ), magMsgYstringSize );
							memcpy ( magMsgZstring, &( port->buffer[magSeparator[1] + 1] ), magMsgZstringSize );

							magMsgXstring[magMsgXstringSize] = '\0';
							magMsgYstring[magMsgYstringSize] = '\0';
							magMsgZstring[magMsgZstringSize] = '\0';

							magnet->raw->values[0] = atoi ( magMsgXstring );
							magnet->raw->values[1] = atoi ( magMsgYstring );
							magnet->raw->values[2] = atoi ( magMsgZstring );

							double mag_s[3];
							double mag_b[3];
							double vecLength;
							for ( j = 0; j < 3; j++ )
							{
								magnet->out->rawFloat[j] = magnet->raw->values[j];
								mag_s[j] = ( double ) ( magnet->raw->values[j] - magnet->set->bias[j] );
							}
							/* transform measurements to the body frame */
							map_vector ( magnet->set->dcm_sb, mag_s, mag_b );

							/* normalize */

							vecLength = sqrt ( ( SQ ( mag_b[0] ) + SQ ( mag_b[1] ) + SQ ( mag_b[2] ) ) );

							if ( vecLength > magnet->set->minStrength )
							{
								magnet->out->itime++;
								magnet->out->magnetStatus = MAGNET_ON;
								magnet->out->lastUpdate = ob->navigation->out->time;

								for ( j = 0; j < 3; j++ )
								{
									magnet->out->field_B[j] = mag_b[j] / vecLength;
								}
							}

						}
						else if ( magCommaIdx == 3 )
						{
							magnet->out->magnetStatus = MAGNET_CALIBRATE;
						}

						magCommaIdx = 0;
						index = index + 2;

					}
					else
					{
						if ( port->buffer[index] == ',' )
						{
							/* we only really expect 2 commas in the message data */
							magSeparator[MIN ( magCommaIdx, 1 )] = index;
							magCommaIdx++;
						}
						index++;
					}

					if ( index < 0 ) index = BUFFERSIZE - 1;

				} /* end loop to read up to last HMR3400 message */

				//clearPort( port, MAX( 0, magMsgEnd - 1 ) );
				//clearPort( port, MAX( 0, index - 3 ) );
				clearPort ( port, MAX ( 0, MAX ( magMsgEnd - 1, port->bytesread - 50 ) ) );

			}
			else
			{

				/* look for a header */
				while ( index <= port->bytesread - MAGSIZE && !done && ( magnet->set->magnetType != 1 ) )
				{

					if ( port->buffer[index + MAGSIZE - 1] == 0x0D )
					{   /* sequence header */

						bf = &( port->buffer[index] );

						memcpy ( magnet->raw, bf, sizeof ( struct magnetRaw_ref ) );
						_swab ( ( char* ) magnet->raw, ( char* ) magnet->raw, sizeof ( struct magnetRaw_ref ) );
						/*_swab( (char *)bf, (char *)magnet->raw, sizeof( struct magnetRaw_ref ) );*/

#if 0        /*first Rmax configuration*/
						magnet->out->itime++;
						magnet->out->magnetStatus = MAGNET_ON;

						int vecLength;
						/*it's mounted backwards */
						magnet->raw->values[0] = -magnet->raw->values[0];
						magnet->raw->values[2] = -magnet->raw->values[2];
						/* normalize */

						vecLength = ( int ) sqrt ( ( double ) ( SQ ( magnet->raw->values[0] )
																										+ SQ ( magnet->raw->values[1] )
																										+ SQ ( magnet->raw->values[2] ) ) );

						if ( vecLength > 0.001 )
						{
							for ( j = 0; j < 3; j++ )
							{
								magnet->out->field_B[j] = ( double ) magnet->raw->values[j] / vecLength;
							}
						}
#else
						double mag_s[3];
						double mag_b[3];
						double vecLength;
						for ( j = 0; j < 3; j++ )
						{
							magnet->out->rawFloat[j] = ( double ) magnet->raw->values[j];
							mag_s[j] = ( double ) ( magnet->raw->values[j] - magnet->set->bias[j] );
						}

						/* transform measurements to the body frame */
						map_vector ( magnet->set->dcm_sb, mag_s, mag_b );

						/* normalize */

						vecLength = sqrt ( ( SQ ( mag_b[0] )
																 + SQ ( mag_b[1] )
																 + SQ ( mag_b[2] ) ) );

						if ( vecLength > magnet->set->minStrength )
						{
							magnet->out->itime++;
							magnet->out->magnetStatus = MAGNET_ON;
							magnet->out->lastUpdate = ob->navigation->out->time;

							for ( j = 0; j < 3; j++ )
							{
								magnet->out->field_B[j] = mag_b[j] / vecLength;
							}
						}
#endif
						index += MAGSIZE - 1;
					}
					index++; /* start seq not found, go to next byte */

					if ( index < 0 ) index = BUFFERSIZE - 1;

				}

				clearPort ( port, index );

			}
		}
	}

	//------------------------------------------------
	// AGL Sensors
	//------------------------------------------------

	updateAglSensors ( sen, NULL, ob->navigation->out->time, ob->navigation->navconfigure->sonUpdateMode );

	/* and now for the GPS... */

	// Novatel 1
	if ( ( gps->set->gpsType == GPS_NOVATEL ) || ( gps->set->backupGpsType == GPS_NOVATEL ) )
	{ // Update if primary or backup
// Call NovAtel initialization and update routines
		timeOutGpsOut ( gps->novatel1->out, gps->set->timeOut, ob->navigation->out->time );
		unsigned char isInCharge = ( ( ( gps->set->gpsType == GPS_NOVATEL ) && ( gps->out->usingBackup == 0 ) ) || // Primary, using primary
																 ( ( gps->set->backupGpsType == GPS_NOVATEL ) && ( gps->out->usingBackup == 1 ) ) ); // Backup, using backup
		updateNovatel ( gps->novatel1, gps->out->outlier, init, data, ob->navigation->out->time, ob->navigation->navconfigure->enableGPS,
										0,0,
										isInCharge, ( gps->set->gpsType == GPS_NOVATEL ) ); // Second check isPrimary
	}

	//prevent autoGpsR to change while nav is running
	static char autoGpsR = gps->set->autoGpsR; static char gpsRPrintFlag = 1;
	if ( autoGpsR != gps->set->autoGpsR )
	{
		if ( ob->navigation->work->navStatus == NAV_RUNNING )
		{
			if ( gpsRPrintFlag )
			{
				logWarning ( "Change GPS antenna mode when nav is running is not allowed" );
				gpsRPrintFlag = 0;
			}
		}
		else
		{//ok to change autoGpsR

			autoGpsR = gps->set->autoGpsR;
			if ( autoGpsR )
			{
				logInfo ( "GPS antenna location (navconfigure.gpsR) is set automatically based on each GPS setting" );
			}
			else
			{
				logInfo ( "GPS antenna location (navconfigure.gpsR) must be set manually" );
			}
			gpsRPrintFlag = 1;
		}
	}

	// Output the appropriate GPS.
	// Note: Latencies are copied here instead of inside the functions since they come from
	//  settings and not from the output directory of each GPS.
	unsigned char currentType = ( gps->out->usingBackup == 1 ) ? gps->set->backupGpsType : gps->set->gpsType;
	switch ( currentType )
	{
		// Copy data here - only thing not going here is the satellite SNR data outputs
	default:
		case GPS_NOVATEL:
		outputGpsOut ( gps->out, gps->novatel1->out, gps->set->registerOutliers );
		gps->out->pLatency = gps->novatel1->set->pLatency;
		gps->out->vLatency = gps->novatel1->set->vLatency;
		if ( autoGpsR )
		{
			mat_copy ( gps->novatel1->set->gpsR, 3, 1, ob->navigation->navconfigure->gpsR );
		}
		break;
	}

	// Determine which GPS should be in charge
	// First, ensure that the backup and primary are different
	if ( gps->set->gpsType == gps->set->backupGpsType )
	{
		gps->set->backupGpsType = -1; // Switch to off
		logError ( "Backup GPS was same as primary - turning backup off" );
	}

	if ( gps->set->backupGpsType == -1 )
	{
		gps->out->usingBackup = 0; // Always on primary if there is only the primary...
	}
	else
	{
		// Check who should be in charge
		// Primary always in charge unless it has failed or is not available...
		// This is all based on the GPS status.

		// Get the primary GPS status
		char primaryGPSStatus = GPS_OFF;
		switch ( gps->set->gpsType )
		{
			default:
			case GPS_NOVATEL:
			primaryGPSStatus = gps->novatel1->out->gpsStatus;
			break;
		}

		// Get the secondary GPS status
		char backupGPSStatus = GPS_OFF;
		switch ( gps->set->backupGpsType )
		{
			default:
			case GPS_NOVATEL:
			backupGPSStatus = gps->novatel1->out->gpsStatus;
			break;
		}
		// The only odd case is when backup GPS status is RTK, but primary isn't.  Otherwise, use primary whenever possible.
		if ( ( backupGPSStatus != GPS_SINGLE ) && ( backupGPSStatus != GPS_DIFF ) && ( backupGPSStatus != GPS_RTK ) && ( backupGPSStatus != GPS_ALIGN ) )
		{
			// Always primary
			gps->out->usingBackup = 0;
		}
		else if ( ( primaryGPSStatus != GPS_SINGLE ) && ( primaryGPSStatus != GPS_DIFF ) && ( primaryGPSStatus != GPS_RTK ) && ( primaryGPSStatus != GPS_ALIGN ) )
		{
			// Always backup since primary is lost
			gps->out->usingBackup = 1;
		}
		else if ( ( ( backupGPSStatus == GPS_RTK ) || ( backupGPSStatus == GPS_ALIGN ) ) && ( primaryGPSStatus == GPS_SINGLE ) )
		{
			// Use backup since backup has RTK, but primary doesn't
			gps->out->usingBackup = 1;
		}
		else
		{
			// Statuses are close enough between the primary and secondary that we default to the primary
			gps->out->usingBackup = 0;
		}
	}

	// Clear any after-update data.  Primarily, this is clearing the data from any GPSs that are
	//  updated,  as the inportant one has already updated the output structure
	// All others
	clearGpsOut ( gps->novatel1->out, 0 ); // Only the itimes

		// This convoluted if statement checks that all the messages required to obtain a valid tightly
		// coupled solution have been received. Need to add cases for different situations.
	if ( ( gps->set->gpsType == GPS_NOVATEL ) && ( gps->set->backupGpsType == -1 ) )
	{
		// This is a special case.  A tightly coupled solution with switching GPSs really doesn't make much sense.
		//  So, assume for a TC solution, there is only 1 GPS, which will be Novatel 1.
		struct senNovatelGps_ref* novatel = gps->novatel1; // TODO!!!!!!!!
		if ( ( novatel->satOut->itimeRange || ( novatel->satOut->itimeTrackStat && novatel->satOut->useTrackStatPsr ) ) &&
				 novatel->satOut->useNvtlSatPos && ( novatel->satOut->lastRangeTime == novatel->satOut->lastSatXYZTime ||
																						 novatel->satOut->lastTrStatTime == novatel->satOut->lastSatXYZTime ) &&
				 ( novatel->satOut->useTrackStatPsr || novatel->satOut->lastRangeTime == novatel->satOut->lastTrStatTime ) )
		{
			novatel->satOut->newPsrs = 1;
			if ( novatel->satOut->useTrackStatPsr )
			{
				novatel->satOut->itimeTC = novatel->satOut->itimeTrackStat;
			}
			else
			{
				novatel->satOut->itimeTC = novatel->satOut->itimeRange;
			}
			gps->out->lastUpdate = ob->navigation->out->time;
			gps->out->gpsStatus = GPS_TC;
		}
	}
}