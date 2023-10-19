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
 * Prof. Eric N. Johnsong
 * http://www.ae.gatech.edu/~ejohnson
 * Tel : 404 385 2519
 * EndCopyright
 ***/
 /***
	* $Id: onboard.cpp,v 1.141 2007-08-27 18:25:13 awu Exp $
	*   top level function calls that execute the navigation and flight controller
	***/
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include "esim/cnsl.h"
#include "esim/util.h"
#include "esim/quat.h"
#include "esim/rand.h"
#include "esim/command.h"
#include "esim/cnsl_ref.h"
#include "esim/db.h" /* this and next for vob */
#include "esim/brwin.h"
#include "rmax/serial.h"
#include "rmax/matrix.h"
#include "rmax/onboard_ref.h"
#include "rmax/controller_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/sensors_ref.h"
#include "rmax/sensors.h"
#include "rmax/datalink.h"
#include "rmax/logger.h"
#include "rmax/logger_ref.h"
#include "rmax/checksum.h"
#include "rmax/onboard.h"
#include "esim/sim_ref.h"
#include "rmax/datalink.h"
#include "rmax/motion_ref.h"  /* to allow use of truth data */
#include "rmax/navigation.h"
#include "rmax/controller.h"

void updateActuator(struct onboard_ref* ob)
{
	struct actuatorInt_ref* act = ob->actuators;
	struct datalinkMessagePWM_ref* pwmout = act->pwmFromUs;
	struct datalinkMessagePWM_ref pwm = { 0 };

	/* PACE vehicle model needs servo commands to be within 0-1000 */
	for ( int i = 0; i < 8; i++ )
	{
		pwm.channel[i] = LIMIT(pwmout->channel[i] - 1000, 0, 1000);
	}

	/* send message */
	pwm.messageID = DATALINK_MESSAGE_PWM;
	pwm.messageSize = sizeof(struct datalinkMessagePWM_ref);

	/* send message */

	datalinkCheckSumEncode((unsigned char*)&pwm, pwm.messageSize);
	writePort(ob->sensors->pwm->p, (char*)&pwm, pwm.messageSize);
}

void updateTasks (void)
{
	struct onboard_ref* ob = &onboard;
	struct navout_ref* nav = ob->navigation->out;
	struct onboardControl_ref* con = ob->control;
	char hadOneLastTime = 0;

	if (ob->sensors->imu->out->packets != 0)
	{  /* it wasn't empty last time we checked */
		hadOneLastTime = 1;
	}

	ob->sensors->imu->out->packets = 0;

	do
	{
		// This is a messaging system to be used with anything that has API access for an underlying messaging system
		//  It's run here because IMU packets come in through this interface for some systems.
		// Start up the port worker or re-initialize it for new configuration data

		readIMU(ob); /* simulated sensors */

		if (ob->sensors->imu->out->newIMUData)
			ob->sensors->imu->out->packets++;


		if (ob->sensors->imu->out->packets > 1)
		{  /* trigger if 2 or more IMU packets were ready */
			ob->navigation->work->overrun++;
			ob->sensors->imu->out->wasntEmpty++;
		}

		if (hadOneLastTime && ob->sensors->imu->out->packets > 0)
		{  /* makes sure buffer was empty on first try */
			ob->sensors->imu->out->wasntEmpty++;
		}

		sensors_update(ob);
		updateNavigation (ob); // TODO: For both sim and hardware
		updateDatalink(ob); // TODO: For hardware

	} while (
		((ob->sensors->imu->p->bytesread >= (int)sizeof(struct imuRaw_ref) && ob->sensors->imu->p->dataSource == PORT_ON) ) 
		&& ob->init == 0);

	/*control*/
	if ( nav->itime != con->work->iLastUpdate )
	{

		if ( nav->itime < con->work->iLastUpdate ) con->work->iLastUpdate = nav->itime; /* just in case */

		if ( ( nav->itime - con->work->iLastUpdate ) * ob->navigation->set->imuDt >= con->work->dtDes )
		{
			con->work->dtFull = ( nav->itime - con->work->iLastUpdate ) * ob->navigation->set->imuDt;
			con->work->dt = MIN ( con->work->dtFull, con->work->dtMax ); /* protect integration scheme */
			con->work->iLastUpdate = nav->itime;
			con->work->time = nav->time;
			updateControl ( ob ); // TODO: For both sim and hardware

			/* send actuator commands */
			updateActuator ( ob );
		}
	}
}

/**
 * @brief This is the real main loop after esim and graphic stuff - TK
 * @param init
*/
void updateOnboard(int init)
{

	struct onboard_ref* ob = &onboard;
	struct navout_ref* nav = ob->navigation->out;
	struct onboardControl_ref* con = ob->control;
	struct logger_ref* loggerSet = &logger;

#ifndef ONBOARD
	if (!ob->run) return;
#endif

	ob->init = init;
	loggerEnableDatalink();

	if (ob->init)
	{

		con->work->iLastUpdate = -1;
		con->work->iLastUpdateCC = -1;

	}

	if (ob->set->init)
	{ /* settings have changed */

		strcpy(ob->datalink->p2->connectTo, ob->set->datalinkHost);
		ob->datalink->p2->init = 1; /* note it's already 1 if this is first time through */
		printf("\n\ndatalinkHost = %s\n\n", ob->datalink->p2->connectTo);

		if (strlen(ob->set->welcomeMessage))
		{
			logInfo(ob->set->welcomeMessage);
			commandExecute("ls root.version");
		}
		if (strlen(ob->set->configDateTime))
		{
			logInfo(ob->set->configDateTime);
		}

		// Allow to initialize logger correctly, and to reinitialize
		if (loggerSet->init == 1)
		{
			initLogger(loggerSet);
			loggerSet->init = 0;
		}

		ob->set->init = 0;
	}

	updateTasks();

	loggerDisableDatalink(); /* important to disable at the end here, as other components may not want this (e.g. GCS) */

}

void initOnboard()
{
	initLogger(&logger);
	loggerExtraDatalinkFunc(datalinkLog);
	cnslExtraPrintFunc(datalinkCnslPrint);
	printf("Onboard:  INIT started\n");
	commandExecute("ls root.version");
	commandLoad("ifconfig", ifconfigCmd,
							 "show interfaces");
	printf("Onboard:  INIT finished\n");

}

void shutdownOnboard()
{
	struct onboard_ref* ob = &onboard;
	printf("Onboard : SHUTDOWN started\n");
	sensors_close(ob);
	datalink_close(ob);
	printf("Onboard : SHUTDOWN finished\n");
	logInfo("> ESim normal exit, goodbye.");
}