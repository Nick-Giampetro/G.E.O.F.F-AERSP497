#ifndef esim_cnsl_h
#define esim_cnsl_h
#if defined(__cplusplus)
extern "C"
{
#endif

#include "esim/cnsl_ref.h"

void commandCnslAdd( int argc, char **argv );
void addCnslButton( char *func, char in[BSIZE][BSIZE] );
int initCnsl( void );
void openCnsl( void );
void updateCnsl( void );
void cnslAddLine( const char *newLine );
void cnslKeyboard( unsigned char key, int x, int y );
void cnslExtraPrintFunc( void (*func)( const char * ) );
void cnslExtraCmdFunc( void (*func)( const char * ) );
void cnslonexit( void );
void commandFileNamePopup( int argc, char **argv );
void commandConfirm( int argc, char **argv );
void fileNamePopupKeyboard( unsigned char key, int x, int y );

#if defined(__cplusplus)
}
#endif
#endif
