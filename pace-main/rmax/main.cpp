#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esim/esim.h"
#include "esim/sim_ref.h"
#include "esim/command.h"
#include "esim/cnsl_ref.h"
#include "rmax/motion_ref.h"
#include "rmax/motion.h"
#include "rmax/si.h"
#include "rmax/gcs.h"
#include "rmax/panel.h"
#include "rmax/scene_ref.h"
#include "rmax/scene.h"
#include "rmax/onboard.h"
#include "rmax/onboard_ref.h"
#include "rmax/ether.h"
#include "rmax/commands.h"
#include "rmax/root_ref.h"
#include "rmax/wdb.h"
#include "rmax/main.h"

void simTick( void ) {

	updateMotion();
	updateSI();

	if ( sim.mode == SIM_MODE_INIT ) {
		updateOnboard( 1 );
	}
	else if ( sim.mode == SIM_MODE_RUN ) {
		updateOnboard( 0 );
	}

	updateSIInputs();
	updateGCS();
}

void drawTick( void ) {
	updateScenes();
	updatePanel();
}

int main( int argc, char** argv ) {

	initEsim( argc, argv );
	loadUserCommands();

	initMotion();

	initOnboard(); // historyInit() #1

	initScenes();
	initPanel();

	initGCS();

	wdbInit();

	commandExecute( "@config" );
	commandExecute ( "multirotor" );
	commandExecute ( "obMultirotor" );

	runEsim();

	// these guys dont really get called because runesim never returns
  // but they will get called in the exit callback before exit
	shutdownOnboard();

	return 0;

}