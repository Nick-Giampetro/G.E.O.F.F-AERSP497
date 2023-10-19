/*** BeginCopyright
 * Copyright 2002, Georgia Institute of Technology, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * UAV Laboratory
 * School of Aerospace Engineering
 * Georgia Institute of Technology
 * Atlanta, GA 30332
 * http://controls.ae.gatech.edu
 *
 * Contact Information:
 * Prof. Eric N. Johnson
 * http://www.ae.gatech.edu/~ejohnson
 * Tel : 404 385 2519
 * EndCopyright
 ***/
/***
 * $Id: commands.cpp,v 1.281 2007/12/14 02:52:00 awu Exp $
 * contains simple C routines to can be used to configure
 * the software. they were initially designed to be executed
 * from the vxworks shell. many are available as esim commands
 ***/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rmax/onboard_ref.h"

#include "rmax/sensors_ref.h"
#include "rmax/gcs_ref.h"

#include "rmax/logger.h"
#include "esim/plwin_ref.h"
#include "esim/brwin_ref.h"
#include "esim/cnsl.h"
#include "esim/util.h"
#include "rmax/scene_ref.h"
#include "esim/db.h"
#include "rmax/motion_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/controller_ref.h"
#include "rmax/realScene_ref.h" // for commandSaveMck
#include <GL/glut.h>
#include "esim/command.h"
#include "rmax/commands.h"

void commandSaveMck( int argc, char **argv ) {
    char buf[2048];
    const char *color = "yellow";

    //wdbadd building latlong lat long  lt   alt  length width  height heading
    FILE *f = 0;
    f = fopen("mckwdb.inp","w");
    if( !f ) {
	logError("cannot open mckwdb.inp to write");
	return;
    }

    for( int i = 0; i < realScene.set->numBuildings; i++ ) {
	struct building_ref *b = realScene.bldg[i];
	switch( b->color ) {
	case BUILDING_WHITE:
	    color = "white";
	    break;
	case BUILDING_RED:
	    color = "red";
	    break;
	case BUILDING_GREEN:
	    color = "green";
	    break;
	case BUILDING_BLUE:
	    color = "blue";
	    break;
	case BUILDING_YELLOW:
	    color = "yellow";
	    break;
	case BUILDING_PURPLE:
	    color = "purple";
	    break;
	case BUILDING_CYAN:
	    color = "cyan";
	    break;
	case BUILDING_BROWN:
	    color = "brown";
	    break;
	case BUILDING_PINK:
	    color = "pink";
	    break;
	default:
	    color = "yellow";
	    break;
	}
	sprintf(buf,"wdbadd building latlong %20.10f %20.10f lt %5.2f %5.2f %5.2f %5.2f %5.2f %s\n",b->latitude,b->longitude, 0.0, b->length, b->width, b->height, b->angle, color);
	fwrite(buf,strlen(buf),1,f);
    }
    fclose(f);
    logInfo("write McKenna Database to mckwdb.inp");
}

void commandSaveEsim( int argc, char **argv ) {

#define SAVESCENE( SCENE, SCENEVAR )    \
	if( (SCENE).open ) {                \
		sprintf(buf,"\n%s.eyePhi       = %f", (SCENEVAR), (SCENE).eyePhi    );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.eyeTheta     = %f", (SCENEVAR), (SCENE).eyeTheta  );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.eyePsi       = %f", (SCENEVAR), (SCENE).eyePsi    );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.eyeLat       = %f", (SCENEVAR), (SCENE).eyeLat    );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.eyeLon       = %f", (SCENEVAR), (SCENE).eyeLon    );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.eyeAlt       = %f", (SCENEVAR), (SCENE).eyeAlt    );            fprintf(of,"%s",buf);   \
	};

#define SAVEWIN( PANEL, PANELVAR )      \
    if( (PANEL).open ) {                \
        glutSetWindow((PANEL).win);     \
        posx = glutGet(GLUT_WINDOW_X);  \
        posy = glutGet(GLUT_WINDOW_Y);  \
        sprintf(buf,"\n%s.x     = %d", (PANELVAR), posx);           fprintf(of,"%s",buf); \
        sprintf(buf,"\n%s.y     = %d", (PANELVAR), posy);           fprintf(of,"%s",buf); \
        sprintf(buf,"\n%s.winw  = %d", (PANELVAR), (PANEL).winw);   fprintf(of,"%s",buf); \
        sprintf(buf,"\n%s.winh  = %d", (PANELVAR), (PANEL).winh);   fprintf(of,"%s",buf); \
    };

#define SAVEBR( BR, BRVAR) \
    if ( (BR).currentDir != NULL ) {                                            \
        sprintf(buf,"\n%s.currentDir = %s", (BRVAR), (BR).currentDir->name); fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.xtype      = %d", (BRVAR), (BR).xtype   );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.xname      = %d", (BRVAR), (BR).xname   );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.xvalue     = %d", (BRVAR), (BR).xvalue  );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.xcomment   = %d", (BRVAR), (BR).xcomment);            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.wtype      = %d", (BRVAR), (BR).wtype   );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.wname      = %d", (BRVAR), (BR).wname   );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.wvalue     = %d", (BRVAR), (BR).wvalue  );            fprintf(of,"%s",buf);   \
		sprintf(buf,"\n%s.wcomment   = %d", (BRVAR), (BR).wcomment);            fprintf(of,"%s",buf);   \
    };

#define SAVEPLVAR( PL, PLVAR ) \
    sprintf(buf,"\n%s.numberOfPlots = %d", (PLVAR), (PL).numberOfPlots);         fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.xvar          = %s", (PLVAR), (PL).xvar);                  fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar0         = %s", (PLVAR), (PL).yvar0);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar1         = %s", (PLVAR), (PL).yvar1);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar2         = %s", (PLVAR), (PL).yvar2);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar3         = %s", (PLVAR), (PL).yvar3);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar4         = %s", (PLVAR), (PL).yvar4);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar5         = %s", (PLVAR), (PL).yvar5);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar6         = %s", (PLVAR), (PL).yvar6);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar7         = %s", (PLVAR), (PL).yvar7);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar8         = %s", (PLVAR), (PL).yvar8);                 fprintf(of,"%s",buf);   \
    sprintf(buf,"\n%s.yvar9         = %s", (PLVAR), (PL).yvar9);                 fprintf(of,"%s",buf);



#ifndef NOGRAPHICS

    FILE *of;
    char fn[1024];
    char buf[1024];
    int currentWindow  = 0;
    int posx = 0;
    int posy = 0;

    if( argc > 1 ) {
        strcpy(fn,  argv[1]);
    } else {
        strcpy(fn, "configEsim.inp");
    }

    currentWindow = glutGetWindow();

    of = fopen( fn, "w" );

    if(of != NULL) {

		SAVEWIN( gcs0Panel, "gcs0Panel" );

        glutSetWindow(cnsl.win);
        posx = glutGet(GLUT_WINDOW_X);
        posy = glutGet(GLUT_WINDOW_Y);
        sprintf(buf,"\n%s.x     = %d", "cnsl", posx);        fprintf(of,"%s",buf); \
        sprintf(buf,"\n%s.y     = %d", "cnsl", posy);        fprintf(of,"%s",buf); \
        sprintf(buf,"\n%s.winw  = %d", "cnsl", cnsl.winw);   fprintf(of,"%s",buf); \
        sprintf(buf,"\n%s.winh  = %d", "cnsl", cnsl.winh);   fprintf(of,"%s",buf); \

		//SAVEWIN( cnsl,  "cnsl"  );

        SAVEWIN( scene0, "scene0" );
        SAVEWIN( scene1, "scene1" );
        SAVEWIN( scene2, "scene2" );

		SAVESCENE( scene0, "scene0" );
		SAVESCENE( scene1, "scene1" );
		SAVESCENE( scene2, "scene2" );


        SAVEWIN( brwin[0], "brwin0" );
        SAVEWIN( brwin[1], "brwin1" );
        SAVEWIN( brwin[2], "brwin2" );

        SAVEWIN( plwin[0], "plwin0" );
        SAVEWIN( plwin[1], "plwin1" );
        SAVEWIN( plwin[2], "plwin2" );

        SAVEBR ( brwin[0], "brwin0" );
        SAVEBR ( brwin[1], "brwin1" );
        SAVEBR ( brwin[2], "brwin2" );

        SAVEPLVAR( plwin[0], "plwin0");
        SAVEPLVAR( plwin[1], "plwin1");
        SAVEPLVAR( plwin[2], "plwin2");

        fclose(of);
        sprintf(buf,"saved windows config to %s",fn);
        logInfo(buf);

    } else {
        sprintf(buf,"\nsaveWins cannot open %s", fn);
        logError(buf);

    }
#else
    logInfo("not graphical esim, nothing to save");
#endif


}

void setupOnboardForMultirotor( void ) {

	onboardSet.type = MODEL_MULTIROTOR;
	onboard.actuators->pwmMap->mixMode = MIX_MULTIROTOR;

	//turn off things that are not on the vehicle
	navconfigure.enableGPS    = 0;
	navconfigure.enableAGL    = 1;
	navconfigure.enableMagnet = 0;
	navconfigure.enableHub    = 0;
	navconfigure.enableAir    = 0;

	navout.wow=1;

	navconfigure.imuR[0] = 0.0/12; //in->ft
	navconfigure.imuR[1] = 0.0/12;
	navconfigure.imuR[2] = 0.0/12;

	navinit.x0[0]  = 0;
	navinit.x0[1]  = 0;
	navinit.x0[2]  = -0.37;
	navset.zgear   = 0.37;
	navset.hground = 1; //height which interpret that we are on ground
	                            //should be slightly higher than zgear
	navinit.ab0[0] = 0;
	navinit.ab0[1] = 0;
	navinit.ab0[2] = 0;
	
	navset.Q[NAV_VX] = 1;
	navset.Q[NAV_VY] = 1;
	navset.Q[NAV_PX] = 0.1;
	navset.Q[NAV_PY] = 0.1;
	
	senSonarData0.sf = 1;
	sonarOut0.alt_min=0.1;
	sonarOut0.alt_max=20.0;
	senSonar0.decodeMode = SONAR_RMAX;
	
	senSonarData0.sonR[0] = 0;
	senSonarData0.sonR[1] = 0;
	senSonarData0.sonR[2] = 0.1;
	senSonarData0.RsonMin = 0.0225;
	navTh.errorLimit = 0.6;
	navTh.nignore = 3;
    
}

void setupOnboardForMultirotorCmd( int argc, char **argv) {
	setupOnboardForMultirotor();
}
void loadUserCommands( void ) {
	commandLoad( "obMultirotor",  &setupOnboardForMultirotorCmd,  "set to onboard code, multirotor");
}
