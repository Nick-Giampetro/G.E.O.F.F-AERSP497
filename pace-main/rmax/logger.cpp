#include "esim/cnsl.h"
#include "rmax/logger_ref.h"
#include "rmax/onboard_ref.h"
#include "esim/esim_ref.h"
#include "esim/sim_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/logger.h"
#include "datalink.h"


#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>
#include <io.h>


struct loggerStatic_ref {
	unsigned char fileLogLevel;
	unsigned char consoleLogLevel;
	unsigned char maxFiles;
	unsigned int  maxFileSizeKb;
	char          dir[200];
	char          fileBaseName[200];
	char          fileBasePath[200];
	char          fileLogPath[200];
	FILE*         fileLogHandle;
	unsigned char logFileId;
    void (*extraDatalinkFunc)(const char *);
	char doDatalink;
#ifdef HAVE_PTHREADS
	pthread_mutex_t lockMut;
#endif
};
static struct loggerStatic_ref loggerStatic;


void loggerExtraDatalinkFunc( void (*func)( const char * ) ) {
	loggerStatic.extraDatalinkFunc = func;
}

void loggerEnableDatalink( void ) { /* send logs that would go to console to the GCS as well */
	loggerStatic.doDatalink = 1;
}

void loggerDisableDatalink( void ) {
	loggerStatic.doDatalink = 0;
}

///////////////////////////////////////////////////////////////////////////////
// Mutex control

static void logger_lock() {
#ifdef HAVE_PTHREADS
	pthread_mutex_lock(&loggerStatic.lockMut);
#endif
}

static void logger_unlock() {
#ifdef HAVE_PTHREADS
	pthread_mutex_unlock(&loggerStatic.lockMut);
#endif
}

unsigned char openLogFile(unsigned char append)
{
    char buf1[200];
	char* pointPtr;

	// An extension may be defined...
	pointPtr = strchr(loggerStatic.fileBaseName, '.');
	if(pointPtr != NULL)
	{
		// Get the base name without the extension
		memcpy(buf1, loggerStatic.fileBaseName, pointPtr - loggerStatic.fileBaseName);
		buf1[pointPtr - loggerStatic.fileBaseName] = '\0';

		// Add the number into the log file name
		sprintf(loggerStatic.fileLogPath, "%s/%s%d%s", loggerStatic.dir, buf1, loggerStatic.logFileId, pointPtr);
	}
	else // No extension defined...
	{
		// Add the number into the log file name
		sprintf(loggerStatic.fileLogPath, "%s/%s%d", loggerStatic.dir, loggerStatic.fileBaseName, loggerStatic.logFileId);
	}

	if(append == 1)
	{
		loggerStatic.fileLogHandle = fopen(loggerStatic.fileLogPath, "a");
	}
	else
	{
		loggerStatic.fileLogHandle = fopen(loggerStatic.fileLogPath, "w");
	}

	if(loggerStatic.fileLogHandle == NULL)
	{
		printf("%s", strerror(errno));
	}
	return (loggerStatic.fileLogHandle == NULL) ? 0 : 1;
}

unsigned char openBaseFile()
{
	size_t readLength;
	int strValue;
	char buf1[200];
	FILE* baseHandle;
	sprintf(buf1,"mkdir %s", loggerStatic.dir);
    #if defined(SUL_OS_WIN32)
    for ( int i=0;i<(int)strlen(buf1);i++ ) {
        if ( buf1[i] == 0x2f ) {
            buf1[i] = 0x5c;
        }
    }
    #endif
    system(buf1);
    sprintf(loggerStatic.fileBasePath, "%s/%s",loggerStatic.dir, loggerStatic.fileBaseName);
	baseHandle = fopen(loggerStatic.fileBasePath, "r");
	if(baseHandle != NULL)
	{
		// Read where we currently are - will always read less than the max buffer size
		readLength = fread(buf1, 1, 200, baseHandle);
		if(readLength > 0)
		{
			strValue = atoi(buf1);
			if((strValue > 0) && (strValue <= loggerStatic.maxFiles))
			{
				loggerStatic.logFileId = strValue;
			}
			else
			{
				// Bad data - start at file 1 and location zero
				loggerStatic.logFileId = 1;
			}
		}
		else
		{
			// Empty file - start at file 1 and location zero
			loggerStatic.logFileId = 1;
		}
		return 1;
	}
	else
	{
		// Non-existent file - start at file 1 and location zero
		loggerStatic.logFileId = 1;
	}

	// Close and reopen for writing since there is not a mode that allows reading and writing anywhere in the file without deleting contents on open
	if(baseHandle != NULL)
	{
		fclose(baseHandle);
		baseHandle = NULL;
	}
	// Open for writing
	baseHandle = fopen(loggerStatic.fileBasePath, "w");
	if(baseHandle != NULL)
	{
		// Print the ID of the log file
		fprintf(baseHandle, "%d", loggerStatic.logFileId);
		fflush(baseHandle);
		// Only need to open when updating
		fclose(baseHandle);
		return 1;
	}
	else
	{
		return 0;
	}
}

void updateLogfile()
{
	struct stat fileBuf;
	FILE* baseHandle;

	// Check whether to cycle to the next file
	stat(loggerStatic.fileLogPath, &fileBuf);
	unsigned int loc = ftell(loggerStatic.fileLogHandle);

	// Switch to new file if necessary
	if(loc >= (loggerStatic.maxFileSizeKb*1024))
	{
		// Close the file
		fclose(loggerStatic.fileLogHandle);

		// Update the ID
		if(loggerStatic.logFileId++ > loggerStatic.maxFiles)
		{
			loggerStatic.logFileId = 1;
		}

		// Open the new file - overwrite
		openLogFile(0);

		// Update the base file
		// Open for writing
		baseHandle = fopen(loggerStatic.fileBasePath, "w");
		if(baseHandle != NULL)
		{
			// Print the ID of the log file
			fprintf(baseHandle, "%d", loggerStatic.logFileId);
			fflush(baseHandle);
			// Only need to open when updating
			fclose(baseHandle);
		}
	}
}

void loggerOnExit()
{
	// Determine whether to close first
	if(loggerStatic.logFileId != 0)
	{
		// Close the files
		if(loggerStatic.fileLogHandle != NULL)
		{
			fflush(loggerStatic.fileLogHandle);
			fclose(loggerStatic.fileLogHandle);
			loggerStatic.fileLogHandle = NULL;
		}

		// Zero out the file ID
		loggerStatic.logFileId = 0;
	}
}

void initLogger( struct logger_ref *logSet )
{
	unsigned char opened = 0;

#ifdef HAVE_PTHREADS
	pthread_mutex_init(&(loggerStatic.lockMut), NULL);
#endif
	// Thread safety if using multithreaded systems
	logger_lock();

	// First, shut down if necessary
	loggerOnExit();

	// Save settings to static structure
	loggerStatic.fileLogLevel = logSet->fileLogLevel;
	loggerStatic.consoleLogLevel = logSet->consoleLogLevel;
	loggerStatic.maxFiles = logSet->maxFiles;
	loggerStatic.maxFileSizeKb = logSet->maxFileSizeKb;
	loggerStatic.doDatalink = 0;
	memcpy(loggerStatic.dir, logSet->dir, strlen(logSet->dir));
	memcpy(loggerStatic.fileBaseName, logSet->fileBaseName, strlen(logSet->fileBaseName));

	// Only open files for logging if necessary
	if(loggerStatic.fileLogLevel > 0)
	{
		// Read the base file
		opened = openBaseFile();

		if(opened == 1)
		{
			// Finally, re-initialize the log file - append at first
			opened = openLogFile(1);
			if(opened == 0)
			{
				printf("Logger: Failed to initialize logging file - unable to start logging to file");

				// Nothing is open
				loggerStatic.logFileId = 0;
			}
		}
		else
		{
			printf("Logger: Failed to initialize logging base file - unable to start logging to file");
			// Nothing is open
			loggerStatic.logFileId = 0;
		}
	}
	else
	{
		printf("Logger: No file logging defined\n");
	}

	// Thread safety if using multithreaded systems
	logger_unlock();
}

void logLevelToStr(char* buf, unsigned char logLevel)
{
    switch (logLevel)
    {
        case LOG_LEVEL_CRITICAL:
            sprintf(buf, "Critical");
			break;
		case LOG_LEVEL_ERROR:
			sprintf(buf, "Error");
			break;
		case LOG_LEVEL_WARNING:
			sprintf(buf, "Warning");
			break;
		case LOG_LEVEL_INFO:
			sprintf(buf, "Info");
			break;
		case LOG_LEVEL_DEBUG:
			sprintf(buf, "Debug");
			break;
		case LOG_LEVEL_VERBOSE:
			sprintf(buf, "Verbose");
			break;
		default:
			sprintf(buf, "Unknown");
    }
}

///////////////////////////////////////////////////////////////////////////////
// Logging functions
///////////////////////////////////////////////////////////////////////////////

void logEvent( const char* filename, const char* functionName, unsigned char logLevel, unsigned char addToCnsl, const char* desc )
{
	// Thread safety if using multithreaded systems
	logger_lock();

	if(logLevel <= loggerStatic.consoleLogLevel)
	{
		// Log to the console (also sends to ground if onboard and set up to do so)

		// important to supress the extra "print" stuff, we'll do it ourselves here (also avoids infite loop, as extraPrint _will_ call this function)
		if( addToCnsl ) {
			int savedExtraPrint;
			savedExtraPrint = cnsl.doExtraPrint;
			cnsl.doExtraPrint = 0;
			cnslAddLine(desc);
			cnsl.doExtraPrint = savedExtraPrint;
		}

		/* now send to ground console as well */
		if( loggerStatic.doDatalink && loggerStatic.extraDatalinkFunc != NULL ) {
			loggerStatic.extraDatalinkFunc( desc );
		}
	}

	if((loggerStatic.logFileId != 0) && (logLevel <= loggerStatic.fileLogLevel))
	{
		double time = 0.0;
		char level[10];

#ifdef ONBOARD
		// Use onboard time
		struct onboard_ref* ob = &onboard;
		time = ob->navigation->out->time;
#else
		// Use sim time - even for onboard2, since need to avoid referencing onboard2 includes in this file
		struct esim_ref* es = &esim;
		time = es->sim->time;
#endif

		// File is initialized and log level is being logged
		// Always create a new line in case the caller did not
		logLevelToStr(level, logLevel);
		fprintf(loggerStatic.fileLogHandle, "(t=%10.2f) %s [%s:%s] %s\n", time, level, filename, functionName, desc);
		fflush(loggerStatic.fileLogHandle);

		// Update the log file
		updateLogfile();
	}

	// Thread safety if using multithreaded systems
	logger_unlock();
}

