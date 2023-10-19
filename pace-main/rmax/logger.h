/**
 * Logging utility, combining file logging capabilities with console prints and datalink logging to GCS
 * This utility is thread-safe.
 *
 * @author Joel Dunham - joel.dunham@gatech.edu
 * @date 2016/06/25
 *
 */

#ifndef RMAX_LOGGER_H
#define RMAX_LOGGER_H

#include <stdio.h>
#include <stdlib.h>
#include <rmax/logger_ref.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes the logger, setting up log settings, etc.  If the logger is
 *  already running, only updates settings such as log level
 */
void initLogger( struct logger_ref *logSet );

void loggerExtraDatalinkFunc( void (*func)( const char * ) );
void loggerEnableDatalink( void );
void loggerDisableDatalink( void );

/**
 * Log function plus convenience functions
 */
void logEvent( const char* filename, const char* functionName, unsigned char logLevel, unsigned char addToCnsl, const char* desc );

#define logCritical(          desc ) logEvent(__FILE__, __FUNCTION__, LOG_LEVEL_CRITICAL, 1, desc )
#define logError(             desc ) logEvent(__FILE__, __FUNCTION__, LOG_LEVEL_ERROR,    1, desc )
#define logWarning(           desc ) logEvent(__FILE__, __FUNCTION__, LOG_LEVEL_WARNING,  1, desc )
#define logInfo(              desc ) logEvent(__FILE__, __FUNCTION__, LOG_LEVEL_INFO,     1, desc )
#define logDebug(             desc ) logEvent(__FILE__, __FUNCTION__, LOG_LEVEL_DEBUG,    1, desc )
#define logVerbose(           desc ) logEvent(__FILE__, __FUNCTION__, LOG_LEVEL_VERBOSE,  1, desc ) 
#define logAtLevel( logLevel, desc ) logEvent(__FILE__, __FUNCTION__, logLevel,           1, desc )

#ifdef __cplusplus
}
#endif


#endif //RMAX_LOGGER_H
