#ifndef esim_brwin_h
#define esim_brwin_h

#if defined(__cplusplus)
extern "C"
{
#endif

void commandBrwin( int argc, char **argv );
int initBrwin( int window );
void updateBrwin( void );
void brwinSetValue( Var* currentVar, char *edit );
void brwinSetValueUnexpanded( Var *currentVar, char *name, char *edit );
void brwinFormatValue( char *buffer, Var *currentVar );
int brwinFormatValueUnexpanded( char *buffer, Var *currentVar, char *name );
void brwinFormatValueAnywhere( char *buffer, void *data, Var *currentVar );
void *brwinGetDataPointerForVar( Var *currentVar, char *name );
void brwinSetPlotMenus( void );
void brwinGetArrayProperties( char *name, int *dimension, int *array );

#if defined(__cplusplus)
}
#endif



#endif
