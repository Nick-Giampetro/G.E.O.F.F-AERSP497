#ifndef rmax_datalink_h
#define rmax_datalink_h


#if defined(__cplusplus)
extern "C"
{
#endif

int datalink_close( struct onboard_ref *ob );
void updateDatalink( struct onboard_ref *ob );
void *getDirDataPointer( int dirNum, int *size );
void datalinkLog( const char * );
void datalinkCnslPrint( const char * );

#if defined(__cplusplus)
}
#endif



#endif
