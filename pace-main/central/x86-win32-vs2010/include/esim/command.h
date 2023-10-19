#ifndef esim_command_h
#define esim_command_h
#if defined(__cplusplus)
extern "C"
{
#endif


void commandExecute( const char *arg );
void commandLoad( const char *command,
                  void (*f)( int argc, char **argv ), 
                  const char *comment );
void commandRun( int argc, char **argv );
void commandRunfast( int argc, char **argv );
void initEsimCommands( void );
void setGlobalDirReference( const char *, const char * );


#if defined(__cplusplus)
}
#endif
#endif
