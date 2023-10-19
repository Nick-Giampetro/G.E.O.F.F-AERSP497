#ifndef esim_vp_h
#define esim_vp_h

#if defined(__cplusplus)
extern "C"
{
#endif

double readDoubleVP( char *vp );
float readFloatVP( char *vp );
int readIntVP( char *vp );
long readLongVP( char *vp );
char readCharVP( char *vp );

void writeDoubleVP( char *vp, double set );
void writeFloatVP( char *vp, float set );
void writeIntVP( char *vp, int set );
void writeLongVP( char *vp, long set );
void writeCharVP( char *vp, char set );



#if defined(__cplusplus)
}
#endif

#endif

