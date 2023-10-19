#include "esim/cnsl.h"
#include "esim/db.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "esim/brwin.h"
#include "rmax/logger.h"

void commandSaveDir( int argc, char **argv ) {

	int i;
	char varName[VARNAMELENGTH+VARNAMELENGTH+5];
	char valueText[VALUELENGTH+1];
	char fileName[1024];
	time_t current_time;
	struct tm *local_time;
	Dir *dir;
	FILE *fp;

	//////////////////////////////
	// wrong syntax for command //
	//////////////////////////////
	if(argc < 2) {
		logInfo("usage : saveDir <dirname> [filename(no ext.)]");
		return;
	}

	//////////////////////////////////
	// generate the input file name //
	//////////////////////////////////

	if( argc > 2 ) {
		strcpy(fileName, argv[2]);
	} else {
		current_time = time ( NULL );
		local_time = localtime( &current_time );

		sprintf( fileName, "%s_%02d%02d%02d_%02d%02d%02d", 
				 argv[1],
				 (1900 + local_time->tm_year)%100,
				 local_time->tm_mon + 1,
				 local_time->tm_mday,
				 local_time->tm_hour,
				 local_time->tm_min,
				 local_time->tm_sec);
	}
	strcat(fileName,".inp");

	fp = fopen(fileName, "w");
	if(fp == NULL) {
		sprintf(varName,"\nCannot open inp file %s",fileName);
		logError(varName);
	} else {
		dir = findDir(argv[1]);

		if(dir != NULL) {
			for(i = 0; i < dir->size; i++) {
				switch( dir->vars[i].type ) {
				default:
					strcpy(varName,dir->name);
					strcat(varName,".");
					strcat(varName,dir->vars[i].name);
					strcat(varName," = ");
					fprintf(fp,varName);
					brwinFormatValue( valueText, &(dir->vars[i]) );
					fprintf(fp, "%s\n", valueText );
					break;
				case TYPE_GENERIC:
				case TYPE_VOIDPTR:
				case TYPE_DIRPTR:
				case TYPE_FUNCPTR:
					break;
				}
			}
			sprintf(valueText,"%s dir saved to %s", dir->name, fileName);
			logInfo(valueText);
		} else {
			sprintf(varName,"%s dir not found", argv[1]);
			logError(varName);
		}
		fclose(fp);
	}

}

