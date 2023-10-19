#ifndef esim_esim_h
#define esim_esim_h

#if defined(__cplusplus)
extern "C"
{
#endif

int initEsim( int argc, char **argv );
int runEsim( void );
void updateEsim( void ); /* for no graphics capability */
// For a single joystick only (Windows or Linux)
void esimJoystick( unsigned int buttonMask,
		   int x, int y, int z,
		   int r, int u, int v );
// For multiple joystick capability
void directCallJoystick();

void initSim( void );
void stopSim( void );
void startSim( void );
void stepSim( void );
void whenexitSim( void (*function)(void));
unsigned long dirNameHash( const char *name );

#if defined(__cplusplus)
}
#endif

#endif
