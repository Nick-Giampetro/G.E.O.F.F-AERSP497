#ifndef rmax_gcs_h
#define rmax_gcs_h

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rmax/gcs_ref.h"

void initGCS( void );
void updateGCS( void );
struct gcsInstance_ref *gcsActiveInstance( struct gcs_ref * );
struct gcsInstance_ref *gcsGetInstance( struct gcs_ref *, int );
/** Returns whether the requested action can be performed (1=yes, 2=no) */

void gcsAddLine( struct gcsInstance_ref *gi, const char *text );
void gcsAddVehicleName( char *, struct vehicleOutputs_ref * );
/*void gcsSendFile( char *fileName, int fileSize );*/
void remoteCommandToInstance( struct gcsInstance_ref *gi, char* buffer );
void sendDirToInstance(struct gcsInstance_ref *gi, char* dirName);

#if defined(__cplusplus)
}
#endif

#endif

