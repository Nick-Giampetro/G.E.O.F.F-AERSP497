#ifndef rmax_serial_h
#define rmax_serial_h

/** Must declare with C linkage because it is serial.c not .cpp */

#ifdef __cplusplus
extern "C" {
#endif

#include "rmax/onboard_ref.h"
void readPort(  struct serialPort_ref *s );
void writePort( struct serialPort_ref *s, char *buffer, int size );
void writePortText( struct serialPort_ref *s, char *buffer );
void clearPort( struct serialPort_ref *s, int shift );
void closePort( struct serialPort_ref *s );
void portSleep( int msec );
int readMsg( struct serialPort_ref *port, void (*cb)(int id, char *data), int readAll );
void writeMsg(struct serialPort_ref *p, char *buf, int id, int size); 
void ifconfigCmd( int argc, char **argv );

#ifdef __cplusplus
}
#endif


#endif
