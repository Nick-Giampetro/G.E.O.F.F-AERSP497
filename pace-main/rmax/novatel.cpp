/**
 * Handles all interactions with NovAtel GPS.  This is not new code - this code
 *  is just moved from sensors.cpp to make the GPS code more modular.
 */

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "esim/util.h"
#include "rmax/checksum.h"
#include "rmax/controller_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/novatel.h"
#include "rmax/sensors_ref.h"
#include "rmax/serial.h"

#define CRC32_POLYNOMIAL 0xEDB88320L

static unsigned int CRC32Value( int i ) {

    int j;
    unsigned int ulCRC;

    ulCRC = i;
    for ( j=8; j>0; j-- ) {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 )^CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;

}

unsigned int gpsCheckSumCompute( unsigned char *ucBuffer, int ulCount ) {

    unsigned int ulTemp1;
    unsigned int ulTemp2;
    unsigned int ulCRC = 0;
    unsigned int crc_expected = 0;

    while ( ulCount-- != 0 ) {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int)ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1^ulTemp2;
    }
    
    memcpy( &crc_expected, ucBuffer, sizeof(unsigned int));
    if ( ulCRC == crc_expected )
        return( 0 );
    else
        return( ulCRC );

}

/* OEM3
unsigned int gpsCheckSumCompute( unsigned char *buf, int byteCount ) {

  int m;
  unsigned char csum;

  csum = 0;
  for( m=0; m<byteCount; m++ )
    csum ^= buf[m];

  return( csum );

}*/

void configureNovatel(struct senNovatelGps_ref *gps, double time)
{
	//char buffer[200];

	switch( gps->set->configure ) {

	case 1:
		gps->set->time = time;
		gps->set->configure = 2;
		break;

	case 2: /* unlog all */
		if ( gps->set->msgFlag == 0 || time > gps->set->time + 0.5) {
			gps->unlogall->messageLength = 2*sizeof(int);
			gps->unlogall->port = GPS_NOVATEL_THISPORT;

			gps->unlogall->crc = 0;
			gps->unlogall->crc = gpsCheckSumCompute( (unsigned char*)gps->unlogall, sizeof(struct gps_unlogall_ref)-sizeof(int) );
			writePort(gps->p, (char*)gps->unlogall, sizeof(struct gps_unlogall_ref) );

			gps->set->time = time;
			gps->set->msgFlag = 1;
		} else if (gps->set->msgFlag == 2) {
			gps->set->configure = 3;
			gps->set->msgFlag = 0;
		}
		break;

	case 3: /* bestposb */
		if( gps->set->sendBestPos ) {
			if ( gps->set->msgFlag == 0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_BESTPOSB;
				gps->log->trigger = GPS_NOVATEL_ONTIME;
				gps->log->period = gps->set->bestPosPeriod;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 4;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure = 4;
		}
		break;

	case 4: /* bestvelb */
		if ( gps->set->sendBestVel ) {
			if( gps->set->msgFlag == 0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_BESTVELB;
				gps->log->trigger = GPS_NOVATEL_ONTIME;
				gps->log->period = gps->set->bestVelPeriod;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 5;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure = 5;
		}
		break;

	case 5: /* send ephemerides*/
		// Will be used later for sending ephemerides

	case 6: /* send range */
		if ( gps->set->sendRange ) {
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_RANGE;
				gps->log->trigger = GPS_NOVATEL_ONTIME;
				gps->log->period = gps->set->rangePeriod;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 7;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure = 7;
		}
		break;

	case 7:
		if ( gps->set->sendIonUtc ) {
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_IONUTC;
				gps->log->trigger = GPS_NOVATEL_ONCHANGED;
				gps->log->period = 0;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 8;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure = 8;
		}
		break;

	case 8:
		if( gps->set->sendTime == 1 ) {
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_TIME;
				gps->log->trigger = GPS_NOVATEL_ONTIME;
				gps->log->period = gps->set->timePeriod;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 9;
				gps->set->msgFlag = 0;
			}

		} else if( gps->set->sendTime == 2 ) {
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_PSRTIME;
				gps->log->trigger = GPS_NOVATEL_ONTIME;
				gps->log->period = gps->set->timePeriod;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 9;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure = 9;
		}
		break;

	case 9:
		if ( gps->set->sendSatPos ) {
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_SATXYZ;
				gps->log->trigger = GPS_NOVATEL_ONTIME;
				gps->log->period = gps->set->satPosPeriod;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 10;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure = 10;
		}
		break;

	case 10:
		if ( gps->set->sendTrackStat) {
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_TRACKSTAT;
				gps->log->trigger = GPS_NOVATEL_ONTIME;
				gps->log->period = gps->set->trackStatPeriod;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure = 11;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure = 11;
		}
		break;

	case 11:
		if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
			gps->clkadjst->messageLength = sizeof(int);
			gps->clkadjst->clkAdjst = gps->set->clockAdjust;

			gps->clkadjst->crc = 0;
			gps->clkadjst->crc = gpsCheckSumCompute( (unsigned char*)gps->clkadjst, sizeof(struct gps_clkadjst_ref)-sizeof(int) );
			writePort(gps->p, (char*)gps->clkadjst, sizeof(struct gps_clkadjst_ref) );

			gps->set->msgFlag = 1;
			gps->set->time = time;
		} else if (gps->set->msgFlag == 2) {
			gps->set->configure = 12;
			gps->set->msgFlag = 0;
		}
		break;

	case 12:
		if( gps->set->setRtk){
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->rtkdyn->messageLength = sizeof(int);
				gps->rtkdyn->mode = 2;

				gps->rtkdyn->crc = 0;
				gps->rtkdyn->crc = gpsCheckSumCompute( (unsigned char*)gps->rtkdyn, sizeof(struct gps_rtkdyn_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->rtkdyn, sizeof(struct gps_rtkdyn_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag==2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		}else{
			// If using single point - don't set RTK
			gps->set->configure++;
		}
		break;
	case 13: //RTK port mode
		if  (gps->set->setRtk > 0 && gps->set->oem >=7){ //because this command does not exist before oem 7
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
			
				gps->rtkPortMode->messageLength = sizeof(int) + sizeof(int);
	
				gps->rtkPortMode->port = GPS_INTERFACE_THISPORT;
				gps->rtkPortMode->mode = LIMIT(gps->set->setRtk-1,0,1);

				gps->rtkPortMode->crc = 0;
				gps->rtkPortMode->crc = gpsCheckSumCompute( (unsigned char*)gps->rtkPortMode, sizeof(struct gps_rtkPortMode_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->rtkPortMode, sizeof(struct gps_rtkPortMode_ref) );
			
				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag==2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		}else{
			gps->set->configure++;
		}
		break;
	case 14: //roverposb
		if ( gps->set->setRtk  == 2) { //only for ALIGN
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port    = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_ROVERPOSB;
				gps->log->trigger = GPS_NOVATEL_ONCHANGED;
				gps->log->period = 0;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure++;
		}
		break;
	case 15: //headingext2
		if ( gps->set->sendHeadingExt2 && gps->set->setRtk == 2) { //only for ALIGN
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port    = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_HEADINGEXT2B;
				gps->log->trigger = GPS_NOVATEL_ONNEW;
				gps->log->period  = 0;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure++;
		}
		break;
	case 16: //relative position
		if ( gps->set->sendBslnEnu && gps->set->setRtk == 2) { //only for ALIGN
			if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
				gps->log->port    = GPS_NOVATEL_THISPORT;
				gps->log->message = MESSAGE_ALIGNBSLNENUB;
				gps->log->trigger = GPS_NOVATEL_ONNEW;
				gps->log->period  = 0;

				gps->log->crc = 0;
				gps->log->crc = gpsCheckSumCompute( (unsigned char*)gps->log, sizeof(struct gps_log_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->log, sizeof(struct gps_log_ref) );

				gps->set->msgFlag = 1;
				gps->set->time = time;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		} else {
			gps->set->configure++;
		}
		break;
	case 17:
		if( gps->set->enableGlonass == 1){
			if ( gps->set->msgFlag == 0 || time > gps->set->time + 0.5) {
				gps->unlock->messageLength = 2*sizeof(int);
				gps->unlock->system = NOVATEL_GNSS_SYS_GLONASS;

				gps->unlock->crc = 0;
				gps->unlock->crc = gpsCheckSumCompute( (unsigned char*)gps->unlock, sizeof(struct gps_unlockSystem_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->unlock, sizeof(struct gps_unlockSystem_ref) );

				gps->set->time = time;
				gps->set->msgFlag = 1;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		}else{
			gps->set->configure++;
		}
		break;
	case 18:
		if( gps->set->enableQzss == 1){
			if ( gps->set->msgFlag == 0 || time > gps->set->time + 0.5) {
				gps->unlock->messageLength = 2*sizeof(int);
				gps->unlock->system = NOVATEL_GNSS_SYS_QZSS;

				gps->unlock->crc = 0;
				gps->unlock->crc = gpsCheckSumCompute( (unsigned char*)gps->unlock, sizeof(struct gps_unlockSystem_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->unlock, sizeof(struct gps_unlockSystem_ref) );

				gps->set->time = time;
				gps->set->msgFlag = 1;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		}else{
			gps->set->configure++;
		}
		break;
	case 19:
		if( gps->set->enableGalileo == 1){
			if ( gps->set->msgFlag == 0 || time > gps->set->time + 0.5) {
				gps->unlock->messageLength = 2*sizeof(int);
				gps->unlock->system = NOVATEL_GNSS_SYS_GALILEO;

				gps->unlock->crc = 0;
				gps->unlock->crc = gpsCheckSumCompute( (unsigned char*)gps->unlock, sizeof(struct gps_unlockSystem_ref)-sizeof(int) );
				writePort(gps->p, (char*)gps->unlock, sizeof(struct gps_unlockSystem_ref) );

				gps->set->time = time;
				gps->set->msgFlag = 1;
			} else if (gps->set->msgFlag == 2) {
				gps->set->configure++;
				gps->set->msgFlag = 0;
			}
		}else{
			gps->set->configure++;
		}
		break;
	case 20:
		if( gps->set->msgFlag==0 || time > gps->set->time + 0.5 ) {
			gps->interfacemd->messageLength = 4*sizeof(int);
			gps->interfacemd->port = GPS_INTERFACE_THISPORT;
			if (gps->set->useNovatelMsg == 0) {
				gps->interfacemd->rxType = GPS_INTERFACE_RTCA;
			} else {
				gps->interfacemd->rxType = GPS_INTERFACE_NOVATEL;
			}
			gps->interfacemd->txType = GPS_INTERFACE_NOVATEL;
			gps->interfacemd->response = 1;

			gps->interfacemd->crc = 0;
			gps->interfacemd->crc = gpsCheckSumCompute( (unsigned char*)gps->interfacemd, sizeof(struct gps_interfaceMode_ref)-sizeof(int) );
			writePort(gps->p, (char*)gps->interfacemd, sizeof(struct gps_interfaceMode_ref) );

			gps->set->time = time;
			gps->set->msgFlag = 1;
		} else if (gps->set->msgFlag==2) {
			gps->set->configure = 0;
			gps->set->msgFlag = 0;
		}
		break;

	default:
		gps->set->configure = 0;
		gps->set->msgFlag = 0;
		break;

	}
}

void outputGpsOut(struct gpsOut_ref *dest, struct gpsOut_ref *src, char registerOutliers){
	for(unsigned int index = 0; index < 3; ++index) {
		dest->p_b_e_L[index] = src->p_b_e_L[index];
		dest->v_b_e_L[index] = src->v_b_e_L[index];
	}
	dest->heading = src->heading;
	dest->lastUpdate = src->lastUpdate;
	dest->lastUpdateRoverposb = src->lastUpdateRoverposb;
	dest->lastUpdateRoverposbUsed = src->lastUpdateRoverposbUsed;
	dest->gpsStatus = src->gpsStatus;
	dest->gpsAccuracy = src->gpsAccuracy;
	dest->numberOfObsTr = src->numberOfObsTr;
	dest->timeStatus = src->timeStatus;
	dest->milliseconds = src->milliseconds;

	if(registerOutliers == 1) {
		dest->outlier = src->outlier;
	}

	dest->alignAvailable = src->alignAvailable;
	dest->week = src->week;

	dest->averageSnr = src->averageSnr;
	dest->hacc = src->hacc;
	dest->sacc = src->sacc;
	dest->pdop = src->pdop;

	// These ones sum up
	dest->itimeHdg += src->newItimeHdg;
	dest->itimePos += src->newItimePos;
	dest->itimeVel += src->newItimeVel;

	// usingBackup and outlierVel are not set through here.
}

void clearGpsOut(struct gpsOut_ref *src, unsigned char clearAll) {
	// Sum up to keep track of all data unless cleared
	src->itimeHdg += src->newItimeHdg;
	src->itimePos += src->newItimePos;
	src->itimeVel += src->newItimeVel;

	src->newItimeHdg = 0;
	src->newItimePos = 0;
	src->newItimeVel = 0;

	if( clearAll == 1) {
		src->itimePos = 0;
		src->itimeVel = 0;
		src->itimeHdg = 0;
		for(unsigned int index = 0; index < 3; ++index) {
			src->p_b_e_L[index] = 0.0;
			src->v_b_e_L[index] = 0.0;
		}
		src->heading = 0.0;
		src->lastUpdate = 0.0;
		src->lastUpdateRoverposb = 0.0;
		src->lastUpdateRoverposbUsed = 0.0;
		src->gpsStatus = GPS_OFF;
		src->gpsAccuracy = GPS_SINGLE;
		src->numberOfObsTr = 0;
		src->timeStatus = 0;
		src->milliseconds = 0;

		src->outlier = 0;
		src->alignAvailable = 0;
		src->week = 0;
		src->averageSnr = 0;
		src->hacc = 0;
		src->sacc = 0;
		src->pdop = 0;
	}
}

void timeOutGpsOut(struct gpsOut_ref *out, double timeout, double time){
	// Handle timeouts
	if( time > out->lastUpdate + timeout ){
		out->averageSnr = 0.0;
		out->numberOfObsTr = 0;
		out->hacc = 0;
		out->sacc = 0;
		out->pdop = 0;
		out->gpsStatus = GPS_OFF;
	}
    if( time < out->lastUpdate ) out->lastUpdate = -99.0;
    if( time > out->lastUpdateRoverposb + timeout ) out->alignAvailable = 0;
    if( time < out->lastUpdateRoverposb ) out->lastUpdateRoverposb = -99.0;
	if( out->gpsStatus == GPS_SINGLE && out->alignAvailable ) out->gpsStatus = GPS_SINGLE_ALIGN_AVAILABLE;
}

void updateNovatel(struct senNovatelGps_ref *novatel, unsigned char currentOutlierStatus, struct navinit_ref *init,
	struct obDatalink_ref *data, double time, char enableGPS, char safemode, char fltPlnManType,
	unsigned char isInCharge, unsigned char isPrimary)
{

	if ( novatel->set->resetSats ) {
		// NR - need reset the ephemerides...
		novatel->set->resetSats = 0;
	}

	if( novatel->set->configure ) {
		// Configure the NovAtel GPS
		configureNovatel(novatel, time);
	}

	// Reset - If it wasn't handled yet, then it stays as an outlier until it is handled.
	novatel->out->outlier = currentOutlierStatus;

    struct serialPort_ref  *port = novatel->p;

    readPort( port );
	if( port->dataSource != PORT_OFF ) {
		int done = 0;
		int index = 0;
		int notSynched = 0;

		/* the 4 is to avoid searching for messages with
			zero size and smaller */
		while( index <= port->bytesread - GPSHEADERSIZE - 4 && !done ) {

			/* JO
				if( port->buffer[i] == '<'  ) {
					// start of abrivated ascii response
					j=0;
					while((port->buffer[i+j] >= 0x32 || port->buffer[i+j]==0xa || port->buffer[i+j]==0xd) && port->buffer[i+j] < 0x7f
							&& j < TEXTBUFFSIZE && i+j < port->bytesread){
						//continue as long data is ascii
						textbf[j] = port->buffer[i+j];
						j++;
					}

					logInfo(textbf); //print any ascii chars following '<'
					j--;

					if(textbf[j] == 0xa){ //increment i if a full message was found
						i += j;
					}
				}
				*/


			if ( ( port->buffer[index]   == 0xaa ) &&
					( port->buffer[index+1] == 0x44 ) &&
					( port->buffer[index+2] == 0x12 ) ) {

				struct gps_bestposb_ref *posb;
				struct gps_bestvelb_ref *velb;
				char usePacket;

				unsigned char *bf = &(port->buffer[index]);

				memcpy( novatel->header, bf, sizeof( struct gps_header_ref ) );

				if ( novatel->header->messageLength + novatel->header->headerLength + 4 < MAXGPSMESSAGESIZE) { /* check integrity of header */
					if ( MIN( novatel->header->messageLength +
								novatel->header->headerLength + 4,
								MAXGPSMESSAGESIZE) + index  <= port->bytesread ) {
						/* have read in the entire message */
						if ( !gpsCheckSumCompute( bf,
													MIN( novatel->header->messageLength + novatel->header->headerLength,
														MAXGPSMESSAGESIZE ) ) ) {
							if ( (novatel->header->messageType & 0x80) == 0 ) {
								/* Valid log and not a response from the receiver */
								//struct gps_satMeasurement_ref* sm;
								//struct gps_trackEntry_ref* tr;
								//struct gps_gpsephem_ref* ephem;
								char useAlignIfAvailable = 0;

								switch ( novatel->header->messageID ) {
									case MESSAGE_BESTVELB:
									case MESSAGE_PSRVELB:
										usePacket = 0;
										switch( novatel->header->messageID ) {
										case MESSAGE_BESTVELB:  default:
											velb = novatel->bestvelb;
											if( novatel->set->useBestvelb ) usePacket = 1;
											break;
										case MESSAGE_PSRVELB:
											velb = novatel->psrvelb;
											if( novatel->set->usePsrvelb ) usePacket = 1;
											break;
										}
										/* be careful: this message may have an alignment problem on FCS20 */
										memcpy( velb, bf, sizeof( struct gps_bestvelb_ref ) );
										/* redo for byte alignment (header should be ok) */
										velb->solStatus       = *( (int *   )(&bf[GPSHEADERSIZE+0 ]) );
										velb->velType         = *( (int *   )(&bf[GPSHEADERSIZE+4 ]) );
										velb->Latency         = *( (float * )(&bf[GPSHEADERSIZE+8 ]) );
										velb->DifferentialAge = *( (float * )(&bf[GPSHEADERSIZE+12]) );
										velb->HorizontalSpeed = *( (double *)(&bf[GPSHEADERSIZE+16]) );
										velb->Track           = *( (double *)(&bf[GPSHEADERSIZE+24]) );
										velb->VerticalSpeed   = *( (double *)(&bf[GPSHEADERSIZE+32]) );
										velb->reserved        = *( (float * )(&bf[GPSHEADERSIZE+40]) );
										/* end redo for byte alignment */
										if( usePacket ) {
											novatel->out->v_b_e_L[0] = velb->HorizontalSpeed*C_M2FT*cos( velb->Track*C_DEG2RAD );
											novatel->out->v_b_e_L[1] = velb->HorizontalSpeed*C_M2FT*sin( velb->Track*C_DEG2RAD );
											novatel->out->v_b_e_L[2] = -velb->VerticalSpeed*C_M2FT;
											if( velb->solStatus == 0 ) novatel->out->newItimeVel++;
										}
										break;

									case MESSAGE_BESTPOSB:
									case MESSAGE_MASTERPOSB:
									case MESSAGE_ROVERPOSB:
									case MESSAGE_PSRPOSB:
										switch( novatel->header->messageID ) {
										case MESSAGE_BESTPOSB:  default:
											posb = novatel->bestposb;
											break;
										case MESSAGE_MASTERPOSB:
											posb = novatel->masterposb;
											break;
										case MESSAGE_ROVERPOSB:
											posb = novatel->roverposb;
											break;
										case MESSAGE_PSRPOSB:
											posb = novatel->psrposb;
											break;
										}
										/* be careful: this message may have an alignment problem on FCS20 */
										memcpy( posb, bf, sizeof( struct gps_bestposb_ref ) );
										/* redo for byte alignment (header should be ok) */
										posb->solStatus       = *( (int *   )(&bf[GPSHEADERSIZE+0 ]) );
										posb->posType         = *( (int *   )(&bf[GPSHEADERSIZE+4 ]) );
										posb->Latitude        = *( (double *)(&bf[GPSHEADERSIZE+8 ]) );
										posb->Longitude       = *( (double *)(&bf[GPSHEADERSIZE+16]) );
										posb->Altitude        = *( (double *)(&bf[GPSHEADERSIZE+24]) );
										posb->Undulation      = *( (float * )(&bf[GPSHEADERSIZE+32]) );
										posb->DatumID         = *( (int *   )(&bf[GPSHEADERSIZE+36]) );
										posb->LatitudeSD      = *( (float * )(&bf[GPSHEADERSIZE+40]) );
										posb->LongitudeSD     = *( (float * )(&bf[GPSHEADERSIZE+44]) );
										posb->AltitudeSD      = *( (float * )(&bf[GPSHEADERSIZE+48]) );
										memcpy( posb->StationID,  &bf[GPSHEADERSIZE+52], 4 );
										posb->DifferentialAge = *( (float * )(&bf[GPSHEADERSIZE+56]) );
										posb->solutionAge     = *( (float * )(&bf[GPSHEADERSIZE+60]) );
										posb->numberOfObsTr   = *(           (&bf[GPSHEADERSIZE+64]) );
										posb->numberOfL1      = *(           (&bf[GPSHEADERSIZE+65]) );
										posb->numberOfL1AboveMask = *(       (&bf[GPSHEADERSIZE+66]) );
										posb->numberOfL2AboveMask = *(       (&bf[GPSHEADERSIZE+67]) );
										memcpy( posb->reserved,  &bf[GPSHEADERSIZE+68], 4 );
										/* end redo for byte alignment */

											useAlignIfAvailable = 0;

										usePacket = 0;
										switch( novatel->header->messageID ) {
										case MESSAGE_BESTPOSB:  default:
											if( novatel->set->useBestposb ) {
												if( useAlignIfAvailable ) {
													if( time < novatel->out->lastUpdateRoverposbUsed ) novatel->out->lastUpdateRoverposbUsed = -99.0;
													if( time > novatel->out->lastUpdateRoverposbUsed + novatel->set->roverposbTimeOut ) {
														usePacket = 1;
													}//else ignore bestposb (and should use roverposb instead)
												} else {
													usePacket = 1;
												}
											}
											break;
										case MESSAGE_MASTERPOSB:
											if( novatel->set->useMasterposb ) { 
												if( !memcmp( posb->StationID, novatel->set->selfStnID, 4 ) ) { 
													usePacket = 1;
												}
											}
											break;
										case MESSAGE_ROVERPOSB:
											if( novatel->set->useRoverposb ) {
												if( !memcmp( posb->StationID, novatel->set->selfStnID, 4 ) ) { 
													usePacket = 1;
												}
											}
											break;
										case MESSAGE_PSRPOSB:
											if( novatel->set->usePsrposb ) {
												if( useAlignIfAvailable ) {
													if( time < novatel->out->lastUpdateRoverposbUsed ) novatel->out->lastUpdateRoverposbUsed = -99.0;
													if( time > novatel->out->lastUpdateRoverposbUsed + novatel->set->roverposbTimeOut ) {
														usePacket = 1;
													}
												} else {
													usePacket = 1;
												}
											}
											break;
										}

										/* throw away old messages logic */
										if( usePacket && novatel->set->filterByTime ) { 
											if( posb->week == novatel->out->week ) {
												if( posb->milliseconds < novatel->out->milliseconds ) usePacket = 0;
											} else {
												/* logic would need to be rather complex and still have ambiguities for the week roll-over every 20 years */
												/* let's just not include this logic (i.e. throwing out messages) outside of the following straightforward case */
												/* specfiically, where the new message week number is just prior to the old one */
												if( ( posb->week + 1 )%1024 == novatel->out->week ) usePacket = 0;
											}
										}

										if( usePacket ) {

											char previousStatus;

											previousStatus = novatel->out->gpsStatus;

											novatel->out->lastUpdate    = time;
											novatel->out->numberOfObsTr = posb->numberOfL1;
											novatel->out->timeStatus    = posb->timeStatus;
											if( posb->solStatus == 0 ) { /* let's only do time filter on valid solutions */
												novatel->out->milliseconds  = posb->milliseconds;
												novatel->out->week          = posb->week;
											}

											novatel->satOut->undulation = posb->Undulation; // NR - needed for tightly coupled filter.

											novatel->out->p_b_e_L[0] = ( posb->Latitude  - init->datumLat )*C_NM2FT*60;
											novatel->out->p_b_e_L[1] = hmodDeg( posb->Longitude - init->datumLon )*C_NM2FT*60
																	*cos( init->datumLat*C_DEG2RAD );
											novatel->out->p_b_e_L[2] = -( posb->Altitude*C_M2FT - init->datumAlt );

											/* status if overwritten below if nolock */
											switch ( posb->posType ) {
												case 0:
												case 1:
												case 2:  /* page 123 of volume 2 */
												case 3:
												case 16:
												case 18:
												default:
													novatel->out->gpsStatus   = GPS_SINGLE;
													novatel->out->gpsAccuracy = GPS_R_SINGLE;
													break;
												case 17:
												case 20:
													novatel->out->gpsStatus   = GPS_DIFF;
													novatel->out->gpsAccuracy = GPS_R_DIFF;
													break;
												case 32:
												//case 33: //for OEM7 33 is reserved
												case 34:
												case 48:
												case 49:
												case 50:
													novatel->out->gpsStatus   = GPS_RTK;
													novatel->out->gpsAccuracy = GPS_R_RTK;
													break;
											}

											/* status is "align" on panel if this is a roverpos message */
											if( posb == novatel->roverposb && posb->solStatus == 0 ) {
												if( useAlignIfAvailable ) {
													novatel->out->gpsStatus = GPS_ALIGN;
													novatel->out->lastUpdateRoverposbUsed = time;
												}
												novatel->out->lastUpdateRoverposb = time;
											}

											/* tell nav it needs to treat this an outlier (jump) */
											/* Always register outliers internally - determine whether to transmit to actual output
											 *  later based on flag.
											 */
											if( 1 == enableGPS ) {
												if( previousStatus == GPS_NOLOCK || previousStatus == GPS_OFF ) {
													novatel->out->outlier = 1;
												}
												if( previousStatus == GPS_ALIGN && novatel->out->gpsStatus != GPS_ALIGN ) {
													novatel->out->outlier = 1;
												}
												if( previousStatus != GPS_ALIGN && novatel->out->gpsStatus == GPS_ALIGN ) {
													novatel->out->outlier = 1;
												}
											}

											/* pick out the case where we do not have lock */
											if( posb->solStatus == 0 ) {
												novatel->out->newItimePos++;
											} else {
												novatel->out->gpsStatus = GPS_NOLOCK;
											}
										}

										break;

									case MESSAGE_HEADINGB:
										memcpy( novatel->headingb, bf, sizeof( struct gps_headingb_ref ) );
										/* redo for byte alignment (header should be ok) - probably not necessary for this one */
										novatel->headingb->solStatus = *( (int *  )(&bf[GPSHEADERSIZE+0 ]) );
										novatel->headingb->posType   = *( (int *  )(&bf[GPSHEADERSIZE+4 ]) );
										novatel->headingb->length    = *( (float *)(&bf[GPSHEADERSIZE+8 ]) );
										novatel->headingb->heading   = *( (float *)(&bf[GPSHEADERSIZE+12]) );
										novatel->headingb->pitch     = *( (float *)(&bf[GPSHEADERSIZE+16]) );
										novatel->headingb->reserved  = *( (float *)(&bf[GPSHEADERSIZE+20]) );
										novatel->headingb->hdgStdDev = *( (float *)(&bf[GPSHEADERSIZE+24]) );
										novatel->headingb->ptchStdDev = *( (float *)(&bf[GPSHEADERSIZE+28]) );
										memcpy( novatel->headingb->stnID,  &bf[GPSHEADERSIZE+32], 4 );
										novatel->headingb->numberSVs     = *( (&bf[GPSHEADERSIZE+40]) );
										novatel->headingb->numberSolnSVs = *( (&bf[GPSHEADERSIZE+41]) );
										novatel->headingb->nObs          = *( (&bf[GPSHEADERSIZE+42]) );
										novatel->headingb->nMulti        = *( (&bf[GPSHEADERSIZE+43]) );
										novatel->headingb->reserved2     = *( (&bf[GPSHEADERSIZE+44]) );
										novatel->headingb->extSolStat    = *( (&bf[GPSHEADERSIZE+45]) );
										novatel->headingb->reserved3     = *( (&bf[GPSHEADERSIZE+46]) );
										novatel->headingb->sigMask       = *( (&bf[GPSHEADERSIZE+47]) );

										/* end redo for byte alignment */
										if( novatel->set->useHeadingb ) {
											novatel->out->heading = novatel->headingb->heading*C_DEG2RAD - novatel->set->headingOffset;
											if( novatel->headingb->solStatus == 0 )
												novatel->out->newItimeHdg++;
										}
										break;

									case MESSAGE_HEADING2B:
										memcpy( novatel->heading2b, bf, sizeof( struct gps_heading2b_ref ) );
										/* redo for byte alignment (header should be ok) - probably not necessary for this one */
										novatel->heading2b->solStatus = *( (int *  )(&bf[GPSHEADERSIZE+0 ]) );
										novatel->heading2b->posType   = *( (int *  )(&bf[GPSHEADERSIZE+4 ]) );
										novatel->heading2b->length    = *( (float *)(&bf[GPSHEADERSIZE+8 ]) );
										novatel->heading2b->heading   = *( (float *)(&bf[GPSHEADERSIZE+12]) );
										novatel->heading2b->pitch     = *( (float *)(&bf[GPSHEADERSIZE+16]) );
										novatel->heading2b->reserved  = *( (float *)(&bf[GPSHEADERSIZE+20]) );
										novatel->heading2b->hdgStdDev = *( (float *)(&bf[GPSHEADERSIZE+24]) );
										novatel->heading2b->ptchStdDev = *( (float *)(&bf[GPSHEADERSIZE+28]) );
										memcpy( novatel->heading2b->roverStnID,  &bf[GPSHEADERSIZE+32], 4 );
										memcpy( novatel->heading2b->masterStnID, &bf[GPSHEADERSIZE+36], 4 );
										novatel->heading2b->numberSVs     = *( (&bf[GPSHEADERSIZE+40]) );
										novatel->heading2b->numberSolnSVs = *( (&bf[GPSHEADERSIZE+41]) );
										novatel->heading2b->nObs          = *( (&bf[GPSHEADERSIZE+42]) );
										novatel->heading2b->nMulti        = *( (&bf[GPSHEADERSIZE+43]) );
										novatel->heading2b->reserved2     = *( (&bf[GPSHEADERSIZE+44]) );
										novatel->heading2b->extSolStat    = *( (&bf[GPSHEADERSIZE+45]) );
										novatel->heading2b->reserved3     = *( (&bf[GPSHEADERSIZE+46]) );
										novatel->heading2b->sigMask       = *( (&bf[GPSHEADERSIZE+47]) );

										if( novatel->set->useHeading2b ) {
											if( !memcmp( novatel->heading2b->roverStnID, novatel->set->roverStnID, 4 ) ) {
												/* end redo for byte alignment */
												novatel->out->heading = novatel->heading2b->heading*C_DEG2RAD - novatel->set->headingOffset;
												if( novatel->heading2b->solStatus == 0 )
													novatel->out->newItimeHdg++;
											}
										}
										break;

									case MESSAGE_BSLNXYZB:
										memcpy( novatel->bslnxyzb, bf, sizeof( struct gps_bslnxyzb_ref ) );
										novatel->bslnxyzb->Bx   = *( (double *)(&bf[GPSHEADERSIZE+8 ]) );
										novatel->bslnxyzb->By   = *( (double *)(&bf[GPSHEADERSIZE+16]) );
										novatel->bslnxyzb->Bz   = *( (double *)(&bf[GPSHEADERSIZE+24]) );
										novatel->bslnxyzb->BxSD = *( (float * )(&bf[GPSHEADERSIZE+32]) );
										novatel->bslnxyzb->BySD = *( (float * )(&bf[GPSHEADERSIZE+36]) );
										novatel->bslnxyzb->BzSD = *( (float * )(&bf[GPSHEADERSIZE+40]) );
										memcpy( novatel->bslnxyzb->StationID,  &bf[GPSHEADERSIZE+44], 4 );
										novatel->bslnxyzb->numberOfObsTr = *( (&bf[GPSHEADERSIZE+48]) );
										break;

									case MESSAGE_ALIGNBSLNENUB:
										memcpy( novatel->alignbslnenub, bf, sizeof( struct gps_alignbslnenub_ref ) );
										novatel->alignbslnenub->East    = *( (double *)(&bf[GPSHEADERSIZE+8 ]) );
										novatel->alignbslnenub->North   = *( (double *)(&bf[GPSHEADERSIZE+16]) );
										novatel->alignbslnenub->Up      = *( (double *)(&bf[GPSHEADERSIZE+24]) );
										novatel->alignbslnenub->EastSD  = *( (float * )(&bf[GPSHEADERSIZE+32]) );
										novatel->alignbslnenub->NorthSD = *( (float * )(&bf[GPSHEADERSIZE+36]) );
										novatel->alignbslnenub->UpSD    = *( (float * )(&bf[GPSHEADERSIZE+40]) );
										memcpy( novatel->alignbslnenub->RoverID,  &bf[GPSHEADERSIZE+44], 4 );
										memcpy( novatel->alignbslnenub->MasterID, &bf[GPSHEADERSIZE+48], 4 );
										novatel->alignbslnenub->numberOfObsTr = *( (&bf[GPSHEADERSIZE+52]) );
										break;

									case MESSAGE_GPSEPHEM:
										/*ephem = (struct gps_gpsephem_ref*)bf;
										memcpy( novatel->satOut->sats[ephem->prn],
												&(ephem->toe),
												sizeof(struct gps_ephem_ref) );
										novatel->satOut->sats[ephem->prn]->sqrt_1me2 = sqrt(1 - SQ(novatel->satOut->sats[ephem->prn]->ecc) );
										novatel->satOut->sats[ephem->prn]->ephemRecieved = 1;*/
										break;

									case MESSAGE_IONUTC:
										memcpy( novatel->ionutc, bf, sizeof( struct gps_ionutc_ref ) );
										novatel->satOut->ionAlpha[0] = novatel->ionutc->alpha0;
										novatel->satOut->ionAlpha[1] = novatel->ionutc->alpha1;
										novatel->satOut->ionAlpha[2] = novatel->ionutc->alpha2;
										novatel->satOut->ionAlpha[3] = novatel->ionutc->alpha3;
										novatel->satOut->ionBeta[0] = novatel->ionutc->b0;
										novatel->satOut->ionBeta[1] = novatel->ionutc->b1;
										novatel->satOut->ionBeta[2] = novatel->ionutc->b2;
										novatel->satOut->ionBeta[3] = novatel->ionutc->b3;
										novatel->satOut->itimeIonUtc++;
										break;

									case MESSAGE_TIME:
										memcpy(novatel->time, bf, sizeof( struct gps_time_ref ));
										novatel->satOut->lastTimeTime = novatel->header->milliseconds/1000.0;
										novatel->satOut->recieverTime = novatel->satOut->lastTimeTime + novatel->time->offset;
										break;

									case MESSAGE_PSRTIME:
										memcpy(novatel->psrtime, bf, sizeof( struct gps_psrTime_ref ));
										novatel->satOut->lastPsrTimeTime = novatel->header->milliseconds/1000.0;
										novatel->satOut->recieverTime = novatel->satOut->lastPsrTimeTime + novatel->psrtime->offset;
										break;

									/*case MESSAGE_PSRPOS:
										memcpy(novatel->psrpos, bf, sizeof( struct gps_psrPos_ref ) );
										novatel->satOut->lastPsrPosTime = novatel->header->milliseconds/1000.0;
										break;*/

									case MESSAGE_PSRXYZ:
										memcpy(novatel->psrxyz, bf, sizeof( struct gps_psrXYZ_ref ) );
										novatel->satOut->lastPsrXYZTime = novatel->header->milliseconds/1000.0;
										break;

									case MESSAGE_SATVIS:
										if (!novatel->satOut->useNvtlCorr) {
											/*memcpy(novatel->satvis, bf, MIN( novatel->header->messageLength+novatel->header->headerLength+4, sizeof( struct gps_satVis_ref ) ) );
											novatel->satOut->lastSatVisTime = novatel->header->milliseconds/1000.0;
											for (unsigned int visIndex=0; visIndex<MIN(novatel->satvis->numSat,NUM_GPS_SATS);visIndex++) {
												struct gps_satVisEntry_ref* sv = (struct gps_satVisEntry_ref*)(novatel->satvis->data);
												if( sv[visIndex].prn < NUM_GPS_SATS && sv[visIndex].prn >= 0 ) {  only handle novatel satellites 
													novatel->satOut->sats[sv[visIndex].prn]->azimuth = sv[visIndex].az*C_DEG2RAD;
													novatel->satOut->sats[sv[visIndex].prn]->elevation = sv[visIndex].elev*C_DEG2RAD;
												}
											}*/
										}
										break;

									case MESSAGE_SATXYZ:
										memcpy(novatel->satpos, bf, MIN(novatel->header->messageLength+novatel->header->headerLength+4, sizeof( struct gps_satxyz_ref ) ));
										/*novatel->satOut->lastSatXYZTime = novatel->header->milliseconds/1000.0;
										for (int posIndex=0; posIndex<MIN(novatel->satpos->numObs,NUM_GPS_SATS); posIndex++) {
											struct gps_satPosition_ref* sp = (struct gps_satPosition_ref*)(novatel->satpos->positions);
											if (novatel->satOut->useNvtlSatPos) {
												novatel->satOut->sats[sp[posIndex].prn]->satPosECEF[0] = sp[posIndex].X;
												novatel->satOut->sats[sp[posIndex].prn]->satPosECEF[1] = sp[posIndex].Y;
												novatel->satOut->sats[sp[posIndex].prn]->satPosECEF[2] = sp[posIndex].Z;
												novatel->satOut->sats[sp[posIndex].prn]->deltaTsv = sp[posIndex].clkCor/C_SOL;
											}
											if (novatel->satOut->useNvtlCorr) {
												novatel->satOut->sats[sp[posIndex].prn]->ionCor  = sp[posIndex].ionCor;
												novatel->satOut->sats[sp[posIndex].prn]->tropCor = sp[posIndex].tropCor;
											}
										}*/
										break;

									case MESSAGE_TRACKSTAT:
										memcpy(novatel->trstat, bf, MIN( novatel->header->messageLength+novatel->header->headerLength+4, sizeof( struct gps_trackstat_ref ) ) );

										/*novatel->satOut->trackUpdateDt = novatel->header->milliseconds/1000.0 - novatel->satOut->lastTrStatTime;
										novatel->satOut->lastTrStatTime = novatel->header->milliseconds/1000.0;
										tr = (struct gps_trackEntry_ref*)(novatel->trstat->data);
										if (novatel->satOut->useTrackStatPsr) {
											memset(novatel->satOut->obsPrns, -1, 12);
										}

										for (int itrack=0,obsIndex=0; itrack<MIN(novatel->trstat->numChannels,MAX_SAT_TRACK); itrack++) {
											if ( tr[itrack].reject != 99 && (tr[itrack].chTrStatus & 0x00200000) == 0 ) {
												novatel->satOut->sats[tr[itrack].prn]->reject = tr[itrack].reject;
												novatel->satOut->sats[tr[itrack].prn]->psrRes = tr[itrack].psrRes;
												novatel->satOut->sats[tr[itrack].prn]->weight = tr[itrack].weight;

												if (novatel->satOut->useTrackStatPsr) {
													novatel->satOut->sats[tr[itrack].prn]->pseudorange = tr[itrack].psr;
													novatel->satOut->sats[tr[itrack].prn]->C2No = tr[itrack].C2No; // SNR
													novatel->satOut->sats[tr[itrack].prn]->lastUpdate = (float)(novatel->satOut->lastTrStatTime);
													novatel->satOut->numObs = obsIndex;
													novatel->satOut->obsPrns[obsIndex++] = (char) tr[itrack].prn;
												}
											}
										}
										novatel->satOut->itimeTrackStat++;*/
										break;

										/* OEM3
											case MESSAGE_VLHB:
											memcpy( novatel->vlhb, bf, sizeof( struct gps_vlhb_ref ) );
											novatel->out->v_b_e_L[0] = novatel->vlhb->HorizontalSpeed*C_M2FT*cos( novatel->vlhb->Track*C_DEG2RAD );
											novatel->out->v_b_e_L[1] = novatel->vlhb->HorizontalSpeed*C_M2FT*sin( novatel->vlhb->Track*C_DEG2RAD );
											novatel->out->v_b_e_L[2] = -novatel->vlhb->VerticalSpeed*C_M2FT;
											if( novatel->vlhb->VelocityStatus < 5 )
											gps->out->newItimeVel++;
											break;
											case MESSAGE_PVAB:
											memcpy( novatel->pvab, bf, sizeof( struct gps_pvab_ref ) );
											break;
											case MESSAGE_CDSB:
											memcpy( novatel->cdsb, bf, sizeof( struct gps_cdsb_ref ) );
											break;
											case MESSAGE_PRTKB:
											memcpy( novatel->prtkb, bf, sizeof( struct gps_prtkb_ref ) );
											gps->out->p_b_e_L[0] = ( novatel->prtkb->Latitude  - init->datumLat )*C_NM2FT*60;
											gps->out->p_b_e_L[1] = hmodDeg( novatel->prtkb->Longitude - init->datumLon )*C_NM2FT*60
											*cos( init->datumLat*C_DEG2RAD );
											gps->out->p_b_e_L[2] = -( novatel->prtkb->Altitude*C_M2FT - init->datumAlt );
											switch( novatel->prtkb->PositionType ) {
											case 0:
											default:
											gps->out->gpsStatus   = GPS_NOLOCK;
											gps->out->gpsAccuracy = GPS_R_SINGLE;
											break;
											case 1:
											gps->out->gpsStatus   = GPS_SINGLE;
											gps->out->gpsAccuracy = GPS_R_SINGLE;
											gps->out->newItimePos++;
											break;
											case 2:
											case 5:
											gps->out->gpsStatus   = GPS_DIFF;
											gps->out->gpsAccuracy = GPS_R_DIFF;
											gps->out->newItimePos++;
											break;
											case 3:
											case 4:
											gps->out->gpsStatus   = GPS_RTK;
											gps->out->gpsAccuracy = GPS_R_RTK;
											gps->out->newItimePos++;
											break;
											}
											break;*/

									default:
										/* unrecongized type */
										break;
								}
							} else {
								//char buf[200];
								memcpy( novatel->response, bf, MIN( novatel->header->messageLength+novatel->header->headerLength+4, sizeof( struct gps_cmdResponse_ref ) ) );
								switch ( novatel->header->messageID ) {
									case MESSAGE_LOG:
									case MESSAGE_UNLOGALL:
									case MESSAGE_CLOCKADJUST:
									case MESSAGE_INTERFACEMODE:
									case MESSAGE_RTKDYNAMICS:
									case MESSAGE_RTKPORTMODE:
										if (novatel->response->responseID == 1) {
											// good request
											novatel->set->msgFlag = 2;
										} else {
											novatel->badPackets++;
											// we have a problem
											/*novatel->response->response[novatel->header->messageLength-4] = '\0';
											sprintf(buf, "novatel:%s", novatel->response->response);
											logInfo(buf);*/
										}
										break;

									case RTCAOBS_RESPONSE_DEBUG:
									case RTCAEPHEM_RESPONSE_DEBUG:
									case RTCAREF_RESPONSE_DEBUG:
									case RTCA1_RESPONSE_DEBUG:
										/* The above labels are *_DEBUG because for some reason the message IDs for
											the response does not match that of the actual message. These are the
											responses to the differential corrections, we don't have to do anything.*/
										break;

									default:
										novatel->badPackets++;
										/*sprintf(buf, "novatel: recieved unhandled response with MsgID - %d", novatel->header->messageID);
										logWarning(buf);
										novatel->response->response[novatel->header->messageLength-4] = '\0';
										sprintf(buf, "novatel:%s", novatel->response->response);
										logWarning(buf);*/
										break;
								}
							}
						} else {
							novatel->badPackets++;
						}

						index += MIN( novatel->header->messageLength +
										novatel->header->headerLength + 4,
										MAXGPSMESSAGESIZE ) - 1;

					} else {
						index--;
						done = 1;
					}
				} else {
					index += MAXGPSMESSAGESIZE;
				}
				notSynched = 0;
			} else {
				notSynched = 1;
			}

			index++; /* start seq not found, go to next byte */
			if( index < 0 ) index = BUFFERSIZE - 1;

		}
		if( notSynched ) index--;
		clearPort( port, index );
	}
}
