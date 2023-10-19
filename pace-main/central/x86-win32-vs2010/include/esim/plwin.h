#ifndef esim_plwin_h
#define esim_plwin_h

#if defined(__cplusplus)
extern "C"
{
#endif

void commandPlwin( int argc, char **argv );
void commandPlot( int argc, char **argv );
int initPlwin( int window );
void updatePlwinData( void );
void updatePlwinGraphics( void );
void plwinSetVar( int window, int value, char *varname );
double plwinReadDataValue( Var *vp, void *data );

#if defined(__cplusplus)
}
#endif

#endif
