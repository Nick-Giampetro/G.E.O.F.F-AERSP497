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
 * $Id: serial.cpp,v 1.17 2007-07-19 21:17:58 kannan Exp $
 * contains the shared memory interface and generic routines
 * to read and write data packets, irrespective of whether
 * it goes through shared memory, serial or udp
 ***/
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "esim/util.h"
#include "rmax/onboard_ref.h"
#include "rmax/root_ref.h"
#include "rmax/ether.h"
#include "rmax/serial.h"

#include "rmax/logger.h"
#include "rmax/si_ref.h"  /* for emulation capability */


static void ifconfigDir( Dir *parent ) {

	/* find all ports */

	if( !strcmp( "serialPort_ref", parent->type ) ) {

		char buffer[400];
		struct serialPort_ref *s = (struct serialPort_ref *)(parent->data);

		/* found a port */

		if( s->init == 0 ) { /* only show initialized ports */
			switch( s->dataSource ) {
			default:
			case PORT_OFF: /* show nothing for this case */
				break;
			case PORT_ON:
				if( s->useSerial ) {
					sprintf( buffer, "> %s: Serial Port=%s %d baud Open=%d tx=%u rx=%u", 
						parent->name, s->name, s->baud, s->portIsOpen, s->sent, s->received );
					logInfo( buffer );
				} else if( s->useTcp && s->useSock ) {
                    sprintf( buffer, "> %s: TCP %s:%d -> %s:%d Open=%d tx=%u rx=%u", 
						parent->name, s->myname, s->myport, s->remotename, s->remoteport, s->portIsOpen, s->sent, s->received );
					logInfo( buffer );
				} else if( s->useSock ) {
					sprintf( buffer, "> %s: UDP %s:%d -> %s:%d Open=%d tx=%u rx=%u", 
						parent->name, s->myname, s->myport, s->remotename, s->remoteport, s->portIsOpen, s->sent, s->received );
					logInfo( buffer );
				}
				break;
			case PORT_SITL:
				sprintf( buffer, "> %s: si_port=%d tx=%u rx=%u", parent->name, s->si_port, s->sent, s->received );
				logInfo( buffer );
				break;
			case PORT_PLAYBACK:
				sprintf( buffer, "> %s: playback", parent->name );
				logInfo( buffer );
				break;
			}
		}
		
    } else {

		Var *v;
		int i;

		for( i=0; i<parent->size; i++ ) {
			v = &(parent->vars[i]);

			if( v->type == TYPE_DIR ) { /* is a subdirectory */
				ifconfigDir( v->child );
			}

		}
	}

}


void ifconfigCmd( int argc, char **argv ) {

	ifconfigDir( &root_dir );

}


void readFromFile( struct serialPort_ref *s ) {

#ifndef FLIGHTCODE
	int ret = 0;
	int toread = 0;

	if( 1 == s->init ) {
		if( s->readFileFd != -1 && s->readFileFd != 0 ) {
			fclose((FILE*)s->readFileFd);
		}
		s->readFileOpen = 0;
	}

    if( s->readFileOpen == 0 ) {

		char fileName[500];

		sprintf( fileName, s->readFile, s->port );

		s->readFileFd = (PTR2INTEGER)fopen( fileName, "r+b" );

		s->readFileOpen = 1;

		if( s->readFileFd != -1 && s->readFileFd != 0 ) {

			printf( " serial:  %s open for playback\n", fileName );

		} else {

			printf( " serial: unable to open %s for playback\n", fileName );

		}

    }

	if( s->readFileFd != -1 && s->readFileFd != 0 && !s->init ) {
		toread = MIN( s->readChunk, BUFFERSIZE - s->bytesread );
		ret    = fread( &(s->buffer[s->bytesread]), 1,  toread, (FILE*)s->readFileFd );
		// the end of file has come
		if( ret < toread ) {
			fclose((FILE*)s->readFileFd);
			s->readFileOpen = -1; // this indicates that file is closed and do not try to reopen it.
			s->readFileFd   = -1;
		}

		s->bytesread += MAX(ret,0); //ret could be -1 so make sure you do not subtract
		s->received  += MAX(ret,0);

	}
#endif

}


static void initPort( struct serialPort_ref *s ) {

	/* safety check to make sure port is not negative */
	if( s->port <= 0 ) {
		s->port = 1;
	}

#ifdef FLIGHTCODE

	/*struct termios termios_p;*/
	if( s->dataSource == PORT_SITL )   /* no emulation capability onboard */
		s->dataSource = PORT_ON;

	sprintf( s->name, s->nameFmt, s->port - 1 );

#else

	struct sitlPorts_ref *sitl = &sitlPorts;

#endif

	// Determine whether to change the connection mode
	if(s->useSerial == 1)
	{
		s->newConnectionMode = MODE_SERIAL;
	}
	else if(s->useSock == 1)
	{
		s->newConnectionMode = (s->useTcp == 1) ? MODE_TCP : MODE_UDP;
	}
	else if (s->useI2C == 1)
	{
		s->newConnectionMode = MODE_I2C;
	}
	else if(s->newConnectionMode == 0)
	{
		// Don't change the connection mode; just re-initialize
		s->newConnectionMode = s->connectionMode;
	}
	// else: do nothing - the new connection mode can be set directly if all other flags are zero

	closePort(s);		/*this statement closes the port first if initialized */

	// Set to now open the new mode
	s->connectionMode = s->newConnectionMode;
	s->newConnectionMode = 0;

	switch( s->dataSource ) {

	case PORT_ON:
		// take care of opening serial
		if(s->connectionMode == MODE_SERIAL) {
			openCom(s);
		}
		// take care of opening sock
		else if((s->connectionMode == MODE_UDP) || (s->connectionMode == MODE_TCP)) {
			openSock(s);
		}
		break;

#ifndef FLIGHTCODE
	case PORT_SITL:
		// I add the MAX_SITL_PORTS in si.db and also added a check for whether si_port is a valid index, SKK, with a visual studio copiler
		if( s->si_port >= MAX_SITL_PORTS ) {
			printf(" serial: si.c valid si_port range is [0,%d] but %d was given for si_port\n",
				MAX_SITL_PORTS-1, s->si_port);
		} else {
			if( ( sitl->con[s->si_port][0] != (void *)s ) &&
				( sitl->con[s->si_port][1] != (void *)s )) {
				if( sitl->con[s->si_port][0] == 0 ) {

					sitl->con[s->si_port][0] = (void *)s;
					printf( " serial: Port %dA Emulator Open\n", s->si_port );

				} else if( sitl->con[s->si_port][1] == 0 ) {

					sitl->con[s->si_port][1] = (void *)s;
					printf( " serial: Port %dB Emulator Open\n", s->si_port );

				} else {
					printf( " serial: Port %d Emulator Open FAIL! (too many)\n", s->si_port );
					printf( "  (port %d and port %d have it)\n",
						((struct serialPort_ref *)sitl->con[s->si_port][0])->port,
						((struct serialPort_ref *)sitl->con[s->si_port][1])->port );
				}
			} else {
				printf( " serial: Port %d Already Open\n", s->si_port );
			}
		}

		break;
#endif

	case PORT_PLAYBACK:
		readFromFile( s );
		break;

    default:
#ifndef FLIGHTCODE
        // remove from all sitl ports
        for( int i=0; i<MAX_SITL_PORTS; i++ ){
            if( sitl->con[i][0] == (void *)s ){
                sitl->con[i][0] = 0;
                printf( " serial: Port %dA Emulator Closed\n", i );
            }
            if( sitl->con[i][1] == (void *)s ){
                printf( " serial: Port %dB Emulator Closed\n", i );
                sitl->con[i][1] = 0;
            }
        }
#endif
		break;
	}

	s->bytesread = 0;
	s->init = 0;

}


void saveToFile( struct serialPort_ref *s, unsigned char *buffer, int size ) {
#ifndef VL_ARCH_C6713

  if( s->saveToFile ) {

    if( s->saveFileOpen == 0 ) {

      char fileName[100], dateStuff[100];

      /* Code for modifying filename so as to add Date and Time */
      time_t current_time;
      struct tm *local_time;

        current_time = time( NULL );
        local_time = localtime( &current_time );
        sprintf( dateStuff, "%02d%02d%02d_%02d%02d%02d",
					(1900 + local_time->tm_year)%100,
					local_time->tm_mon + 1,
					local_time->tm_mday,
					local_time->tm_hour,
					local_time->tm_min,
					local_time->tm_sec );

      sprintf( fileName, s->saveFile, dateStuff );
      s->saveFileFd = (PTR2INTEGER)fopen( fileName, "a+b" );

      s->saveFileOpen = 1;

      if( s->saveFileFd != -1 && s->saveFileFd != 0 ) {
		fwrite( buffer, 1, size, (FILE *)s->saveFileFd );
		/*fwrite( buffer, 1, size, (struct _iobuf *)s->saveFileFd );*/ /* SKK 20 june 2002*/

      } else {

		printf( " serial: unable to open save file %s\n", fileName );

      }

    } else {

      if( s->saveFileFd != -1 && s->saveFileFd != 0 ) {

		fwrite( buffer, 1, size, (FILE*)s->saveFileFd );
		/*fwrite( buffer, 1, size, (struct _iobuf *)s->saveFileFd );*//*SKK*/

      }

    }

  }

#endif
}


/** This function can be nilpotently. It always closes a valid file descriptor
 *  If the file descriptor is invalid we do not care anyway -
 */
void closePort( struct serialPort_ref *s )  {

	if(((s->connectionMode == MODE_UDP) || (s->connectionMode == MODE_TCP)) && (s->portIsOpen == 1))
	{
		closeSock(s);
		s->portIsOpen = 0;
		s->bytesread = 0;
	}
	else if((s->connectionMode == MODE_SERIAL) && (s->portIsOpen == 1))
	{
		closeCom(s);
		s->portIsOpen = 0;
		s->bytesread = 0;
	}

}


void readPort( struct serialPort_ref *s ) {

	int newBytes = 0;

	if( 1 == s->init )
		initPort( s );

    // if serial is being read from stream call streamrecorder read function and return
#if HAVE_STREAMRECORDER
    if(s->sridx >= 0 && s->srmode == sulStreamRecorder::READING) {
        sulStreamRecorder::instance()->readPort(s);
        return;
    }
#endif

	/* Read from device only if port is on */
	switch( s->dataSource ) {

	case PORT_ON:
		if( s->bytesread >= BUFFERSIZE || s->bytesread < 0 ) s->bytesread = 0; /* something very wrong... */
		if( (s->connectionMode == MODE_SERIAL) && s->portIsOpen) {
			newBytes = readCom( s, s->buffer + s->bytesread, BUFFERSIZE - s->bytesread );
			newBytes = LIMIT( newBytes, 0, BUFFERSIZE - s->bytesread ); /* extra protection in case return arguments are wierd in error cases */
			if( newBytes > 0 ) {
				saveToFile( s, s->buffer + s->bytesread, newBytes );
#if HAVE_STREAMRECORDER
	            sulStreamRecorder::instance()->writePort(s, (sulbyte*)(s->buffer+s->bytesread), newBytes );
#endif
				s->bytesread += newBytes;
				memcpy( s->bufferView, s->buffer, 8 );
			}
		}
		else if( ((s->connectionMode == MODE_UDP) || (s->connectionMode == MODE_TCP)) && s->portIsOpen) {
#ifdef WIN32
			WinSerial *ws = (WinSerial*)s->serialDevice;
			if(ws) WaitForSingleObject( ws->readLock, INFINITE);
#endif
			newBytes = readSock(s, s->buffer + s->bytesread, BUFFERSIZE - s->bytesread);
#ifdef WIN32
			if(ws) ReleaseMutex(ws->readLock);
#endif
			newBytes = LIMIT( newBytes, 0, BUFFERSIZE - s->bytesread ); /* extra protection in case return arguments are weird in error cases */
			if( newBytes > 0 ) {
				saveToFile( s, s->buffer + s->bytesread, newBytes );
#if HAVE_STREAMRECORDER
				sulStreamRecorder::instance()->writePort(s, (sulbyte*)(s->buffer+s->bytesread),newBytes);
#endif
				s->bytesread += newBytes;
				memcpy( s->bufferView, s->buffer, 8 );
			}
		}
		break;

	case PORT_SITL:
		if( s->bytesread >= BUFFERSIZE || s->bytesread < 0 ) s->bytesread = 0; /* something very wrong... */
		break;

	case PORT_PLAYBACK:
		readFromFile( s );
		break;

	default:
		break;
	}

}

void writePort( struct serialPort_ref *s, char *buffer, int size ) {

       
	int written = 0;

	/* should probably add functionality to check if send buffer is empty... */

#ifndef FLIGHTCODE
	struct sitlPorts_ref *sitl = &sitlPorts;
	struct serialPort_ref *sto;
#endif

	if( 1 == s->init )
		initPort( s );
	


	switch( s->dataSource ) {

	case PORT_ON:
		if( (s->connectionMode == MODE_SERIAL) && s->portIsOpen ) {
			written = writeCom( s, buffer, size );
		}
		else if( ((s->connectionMode == MODE_UDP) || (s->connectionMode == MODE_TCP)) && s->portIsOpen ) {
#ifdef WIN32
			WinSerial *ws = (WinSerial*)s->serialDevice;
			if(ws) WaitForSingleObject( ws->writeLock, INFINITE);
#endif
			written = writeSock( s, buffer, size );
#ifdef WIN32
			if(ws) ReleaseMutex(ws->writeLock);
#endif
		}
		break;

#ifndef FLIGHTCODE
	case PORT_SITL:

		if( s->si_port >= MAX_SITL_PORTS ) {
			printf( " serial:  si.c valid si_port range is [0,%d] but %d was given for si_port, you are accessing memory you should not be\n", 
				    MAX_SITL_PORTS-1, s->si_port );
		}
		
		
		if( ( sitl->con[s->si_port][0] != 0 ) &&
			( sitl->con[s->si_port][0] != (void *)s ) )
			sto = (struct serialPort_ref *)sitl->con[s->si_port][0];
		else if( ( sitl->con[s->si_port][1] != 0 ) &&
			( sitl->con[s->si_port][1] != (void *)s ) )
			sto = (struct serialPort_ref *)sitl->con[s->si_port][1];
		else {
			/*printf( " serial: emulator pair missing!\n" );*/
			sto = s;
		}
		/* SKK The && sto->dataSource checks if the sensor actually wants data
		* from an emulator before it memcopies emulator data over */
		// with regard to stream recording
		// 1) if stream reording is writing or stopped, go ahead and call writePort
		if(    ( sto != s )
			&& ( sto->dataSource == PORT_SITL )
			&& ( sto->baud == s->baud || ( ((s->connectionMode == MODE_UDP) || (s->connectionMode == MODE_TCP)) && ((sto->connectionMode == MODE_UDP) || (sto->connectionMode == MODE_TCP)) ) )
			) {

			// decide here whether bytes should be injected
#if HAVE_STREAMRECORDER
			int shouldInjectBytes = sulStreamRecorder::instance()->shouldInjectBytes(sto, (sulbyte*)buffer,size);
#else
			int shouldInjectBytes = 1;
#endif

			if(shouldInjectBytes) {
			  		
				sto->bytesread = LIMIT( sto->bytesread, 0, BUFFERSIZE );
				size = MIN( size, BUFFERSIZE - sto->bytesread );

				
				if( size > 0 ) {
					memcpy( &(sto->buffer[sto->bytesread]), buffer, size );
					saveToFile( sto, (unsigned char *)buffer, size );

#if HAVE_STREAMRECORDER
					sulStreamRecorder::instance()->writePort(sto, (sulbyte*)buffer,size);
#endif
					sto->bytesread += size;
					sto->received  += size;
					s->sent        += size;
					memcpy( sto->bufferView, sto->buffer, 8 );
				}
			}

		}

		break;
#endif

	default:
		break;
	}

}

void clearPort( struct serialPort_ref *s, int shift ) {

	if( shift != 0 ) {
		if( s->dataSource == PORT_ON || s->dataSource == PORT_SITL || s->dataSource == PORT_PLAYBACK ) {
			if( shift > 0 ) {
				if( shift >= s->bytesread ) {
					s->bytesread = 0;
				} else {
					memmove( s->buffer, &(s->buffer[shift]), s->bytesread - shift );
					/*memcpy( s->buffer, &(s->buffer[shift]), s->bytesread - shift );*/
					s->bytesread -= shift;
				}
				memcpy( s->bufferView, s->buffer, 8 );
			} else {
				s->bytesread = 0;
			}
		}
	}

}


void writePortText( struct serialPort_ref *s, char *buffer ) {

  writePort( s, buffer, strlen( buffer ) );

}


void portSleep( int msec ) {

#ifdef VXWORKS
  taskDelay( msec/16 );  /* not exact milliseconds */
#endif
#ifdef WIN32
  SleepEx( msec, 1 );
#endif
#ifdef LINUX
  /* do something? */
#endif

}


unsigned char portCsumCompute( unsigned char *buf, int byteCount ) {

  int m;
  unsigned char csum;

  csum = 0;
  for( m=0; m<byteCount; m++ )
    csum ^= buf[m];

  return( csum );

}

/*
void portCsumEncode( unsigned char *buf, int byteCount ) {

    struct datalinkHeader_ref *h = (struct datalinkHeader_ref *)buf;

    SERIALMSG_CSUM(bufh->csum  = 0;
    h->hcsum = 0;
    h->csum  = serialCsumCompute( buf, byteCount );
    h->hcsum = serialCsumCompute( buf, sizeof( struct datalinkHeader_ref ) );

}
*/
void writeMsg(struct serialPort_ref *p, char *buf, int id, int size) {

	SERIALMSG_SYNC1(p->hdr) = SERIAL_SYNC1;
	SERIALMSG_SYNC2(p->hdr) = SERIAL_SYNC2;
	SERIALMSG_ID(p->hdr)    = id;
	SERIALMSG_SIZE(p->hdr)  = size ;
	SERIALMSG_CSUM(p->hdr)  = portCsumCompute((unsigned char *)buf,size);
	SERIALMSG_HCSUM(p->hdr) = 0;
	SERIALMSG_HCSUM(p->hdr) = portCsumCompute((unsigned char*)(p->hdr),SERIALMSG_HEADER_SIZE);
	writePort(p,p->hdr,SERIALMSG_HEADER_SIZE);
	writePort(p,buf,size);
}

int readMsg( struct serialPort_ref *port, void (*cb)(int id, char *data), int readAll ) {
	int i;
	struct serialMsgHeader_ref *hdr = (struct serialMsgHeader_ref*)(port->tmp);
	int done = 0;
	int retflag = 0;

	readPort(port);

	for(i = 0; i < port->bytesread && !done; i++) {

		switch(port->seekState) {
		case SERIAL_SEEK_SYNC1:
			if(port->buffer[i] == SERIAL_SYNC1) {
				SERIALMSG_SYNC1(port->tmp) = SERIAL_SYNC1;
				port->seekState = SERIAL_SEEK_SYNC2;
			}
			break;
		case SERIAL_SEEK_SYNC2:
			if(port->buffer[i] == SERIAL_SYNC2) {
				SERIALMSG_SYNC2(port->tmp) = SERIAL_SYNC2;
				port->seekState = SERIAL_SEEK_ID_0;
			} else {
				port->seekState = SERIAL_SEEK_SYNC1;
			}
			break;
		case SERIAL_SEEK_ID_0:
			SERIALMSG_ID_0(port->tmp) = port->buffer[i];
			port->seekState = SERIAL_SEEK_ID_1;
			break;
		case SERIAL_SEEK_ID_1:
			SERIALMSG_ID_1(port->tmp) = port->buffer[i];
			port->seekState = SERIAL_SEEK_SIZE_0;
			break;
		case SERIAL_SEEK_SIZE_0:
			SERIALMSG_SIZE_0(port->tmp) = port->buffer[i];
			port->seekState = SERIAL_SEEK_SIZE_1;
			break;
		case SERIAL_SEEK_SIZE_1:
			SERIALMSG_SIZE_1(port->tmp) = port->buffer[i];
			port->seekState = SERIAL_SEEK_CSUM;
			break;
		case SERIAL_SEEK_CSUM:
			SERIALMSG_CSUM(port->tmp) = port->buffer[i];
			//bufidx = 0;
			port->seekState = SERIAL_SEEK_HCSUM;
			break;
		case SERIAL_SEEK_HCSUM:
			SERIALMSG_HCSUM(port->tmp) = port->buffer[i];
			if(hdr->hcsum == portCsumCompute((unsigned char *)(port->tmp),SERIALMSG_HEADER_SIZE-1))	{
				port->seekState = SERIAL_SEEK_DATA;
			} else {
				printf(" serial: readMsg:hdr csum did not pass\n");
				port->seekState = SERIAL_SEEK_SYNC1;
			}
			if(hdr->size > BUFFERSIZE) {
				port->seekState = SERIAL_SEEK_SYNC1;
				printf(" serial: readMsg:hdr->size > BUFFERSIZE\n");
			}
			break;
		case SERIAL_SEEK_DATA:
			if(i+hdr->size <= port->bytesread) {
				if(hdr->csum == portCsumCompute((unsigned char*)(port->buffer+i),hdr->size)) {
					cb(hdr->id,(char*)(port->buffer + i));
					if(readAll) {
						i+=hdr->size-1; // -1 because i gets incremented in the for loop
						done = 0;
						retflag++;
					} else {
						i+=hdr->size;	// here we do not need the -1 because it will not get executed infor loop
						done = 1;
						retflag = hdr->id;
					}
					//printf("\ngot packet id = %d",hdr->id);

					port->seekState = SERIAL_SEEK_SYNC1;
					//done = 1;
				} else {
					port->seekState = SERIAL_SEEK_SYNC1;

					printf(" serial: readMsg:csum not passed\n");
				}
			} else {
				done =1;	// full packet not yet read
			}
			break;
		default:
			port->seekState = SERIAL_SEEK_SYNC1;
			break;
		};


	}
	clearPort(port, i);

	return retflag;


}
