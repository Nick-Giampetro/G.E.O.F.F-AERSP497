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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <GL/glut.h>
#include "esim/esim.h"
#include "esim/util.h"
#include "esim/quat.h"
#include "esim/sim_ref.h"
#include "esim/command.h"
#include "esim/cnsl_ref.h"
#include "esim/cnsl.h"
#include "esim/rand.h"
#include "rmax/motion_ref.h"
#include "rmax/xforms.h"
#include "rmax/motion.h"
#include "rmax/gear_ref.h"
#include "rmax/si_ref.h"
#include "rmax/generic_ref.h"
#include "rmax/onboard_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/vector.h"
#include "rmax/gcs.h"
#include "rmax/logger.h"
#include "motion_multirotor.h"
#include "rmax/serial.h" //for send true data to another instance of GUST
#include "rmax/si.h"

#define IFLIBEXT(CODE)

static void motion_cross( const double* v1, const double* v2, double* cross ) {

	cross[0] = ( v1[1] * v2[2] ) - ( v1[2] * v2[1] );
	cross[1] = ( v1[2] * v2[0] ) - ( v1[0] * v2[2] );
	cross[2] = ( v1[0] * v2[1] ) - ( v1[1] * v2[0] );

}


/* Dryden Turbulence Model */
static void turbGeneratorInit( struct vehicleMotion_ref* mo ) {

	struct motionTurbulence_ref* work = mo->env->turbulence;

	work->gust_x[0] = 0;
	work->gust_x[1] = 0;
	work->gust_x[2] = 0;
	work->gust_v_G[0] = 0;
	work->gust_v_G[1] = 0;
	work->gust_v_G[2] = 0;
	work->gust_w_G[0] = 0;
	work->gust_w_G[1] = 0;
	work->gust_w_G[2] = 0;
	work->gust_v[0] = 0;
	work->gust_v[1] = 0;
	work->gust_v[2] = 0;
	work->gust_w[0] = 0;
	work->gust_w[1] = 0;
	work->gust_w[2] = 0;

}

static void turbGenerator( struct vehicleSet_ref* set, struct vehicleMotion_ref* mo ) {

	struct env_ref* env = mo->env;
	struct motionTurbulence_ref* work = env->turbulence;

	if ( set->turbulence > 0.0 ) {

		double n, gust_vdot;

		/* determine the characteristic length and turbulence intensity factor.  These low altitude formulas
			come from the Matlab implementation of this model (MIL-F-8785C), altitude less than 1000 feet */

		work->turbSigma[2] = 0.1 * set->turbulence * 15.0 * C_NM2FT / 3600;
		work->turbSigma[1] = work->turbSigma[2] / pow( ( 0.177 + 0.000823 * LIMIT( mo->altitudeAGL, 0, 1000 ) ), 0.4 );
		work->turbSigma[0] = work->turbSigma[1];

		work->turbL[2] = LIMIT( mo->altitudeAGL, 1, 1000 );
		work->turbL[1] = MAX( 1, mo->altitudeAGL / pow( ( 0.177 + 0.000823 * LIMIT( mo->altitudeAGL, 0, 1000 ) ), 1.2 ) );
		work->turbL[0] = work->turbL[1];

		/* compute gusts */

		/* longitudinal */
		work->gust_v_G[0] += set->dtMax * ( work->turbSigma[0] * sqrt( 2.0 * work->turbL[0] / ( C_PI * mo->taseff ) ) * randne() / sqrt( set->dtMax )
																				- work->gust_v_G[0] ) *
			mo->tas / work->turbL[0];
		work->gust_w_G[0] += set->dtMax * ( work->turbSigma[2] * sqrt( 0.8 / mo->taseff ) * pow( C_PI / ( 4.0 * work->span * SQ( work->turbL[2] ) ), 0.166667 ) * randne() / sqrt( set->dtMax )
																				- work->gust_w_G[0] ) *
			C_PI * mo->tas / ( 4.0 * work->span );

		/* lateral */
		n = randne() / sqrt( set->dtMax );
		gust_vdot = -2.0 * mo->tas / work->turbL[1] * work->gust_v_G[1] + work->gust_x[1] +
			work->turbSigma[1] * sqrt( work->turbL[1] * mo->tas / C_PI ) * sqrt( 3 ) / work->turbL[1] * n;
		work->gust_v_G[1] += set->dtMax * gust_vdot;
		work->gust_x[1] += set->dtMax * ( -1.0 * SQ( mo->tas / work->turbL[1] ) * work->gust_x[1] +
																			work->turbSigma[1] * sqrt( work->turbL[1] / ( C_PI * mo->taseff ) ) * SQ( mo->tas / work->turbL[1] ) * n );
		work->gust_w_G[2] += set->dtMax * ( -gust_vdot - work->gust_w_G[2] * mo->tas ) * C_PI / ( 4.0 * work->span );

		/* vertical */
		n = randne() / sqrt( set->dtMax );
		gust_vdot = -2.0 * mo->tas / work->turbL[2] * work->gust_v_G[2] + work->gust_x[2] +
			work->turbSigma[2] * sqrt( work->turbL[2] * mo->tas / C_PI ) * sqrt( 3 ) / work->turbL[2] * n;
		work->gust_v_G[2] += set->dtMax * gust_vdot;
		work->gust_x[2] += set->dtMax * ( -1.0 * SQ( mo->tas / work->turbL[2] ) * work->gust_x[2] +
																			work->turbSigma[2] * sqrt( work->turbL[2] / ( C_PI * mo->taseff ) ) * SQ( mo->tas / work->turbL[2] ) * n );
		work->gust_w_G[1] += set->dtMax * ( +gust_vdot - work->gust_w_G[1] * mo->tas ) * C_PI / ( 4.0 * work->span );

	}
	else {

		work->gust_x[0] = 0;
		work->gust_x[1] = 0;
		work->gust_x[2] = 0;
		work->gust_v_G[0] = 0;
		work->gust_v_G[1] = 0;
		work->gust_v_G[2] = 0;
		work->gust_w_G[0] = 0;
		work->gust_w_G[1] = 0;
		work->gust_w_G[2] = 0;

	}

	build_rotmatrix( mo->xforms->dcm_bl, mo->state->e );
	map_vector( mo->xforms->dcm_bl, work->gust_v_G, work->gust_v );
	map_vector( mo->xforms->dcm_bl, work->gust_w_G, work->gust_w );

	/*st = sin( track );
	ct = cos( track );
	work->gust_v[0] = work->gust_v_G[0]*ct - work->gust_v_G[1]*st;
	work->gust_v[1] = work->gust_v_G[0]*st + work->gust_v_G[1]*ct;
	work->gust_v[2] = work->gust_v_G[2];
	work->gust_w[0] = work->gust_w_G[0]*ct - work->gust_w_G[1]*st;
	work->gust_w[1] = work->gust_w_G[0]*st + work->gust_w_G[1]*ct;
	work->gust_w[2] = work->gust_w_G[2];*/

}

static void f( struct vehicle_ref* v,
							 struct state_ref* s ) {

	struct vehicleSet_ref* set = v->set;
	struct vehicleMotion_ref* mo = v->motion;

	switch ( set->model ) {

		default:
		case MODEL_MULTIROTOR:
		multirotor_f( set, mo, s, sim.dt );
		break;
	}

}

void updateMotion( void ) {

	struct vehicle_ref* v = &vehicle;
	struct vehicleSet_ref* set = v->set;
	struct vehicleMotion_ref* mo = v->motion;
	struct vehicleOutputs_ref* o = v->outputs;
	struct env_ref* env = mo->env;
	struct motionControls_ref* co = mo->controls;
	struct params_gear_ref* pg = mo->gear;
	//struct params_mr_ref   *mr   = v->helicopter->mr;
	struct motionXforms_ref* xf = mo->xforms;
	struct motionImpulse_ref* impulse = mo->impulse;
	struct state_ref* st = mo->state;
	struct state_ref* sd = mo->stateDot;
	struct state_ref* st1 = mo->state1;

	int i, j;
	double* x, * xd, * in, * txy, * fz;
	double xd1[NSTATES];
	/*float df;*/
	double dcm90[3][3];
	double alpha, beta, cal, cbe, sal, sbe;

	/* read joystick */
#ifndef NOGRAPHICS
	glutSetWindow( cnsl.win ); /* need to use a window that will not be closed */
	// Single joystick capability only
		//glutJoystickFunc( esimJoystick, -1 );
		//glutForceJoystickFunc();
#endif
	// Capability for multiple joysticks
	// Disable polling the single joystick method
	glutJoystickFunc( esimJoystick, -1 );
	// Force call the multiple joystick method
	directCallJoystick();

	if ( !v->run ) return;

	if ( sim.mode == SIM_MODE_INIT ) {

		/* set initial conditions */
		IFLIBEXT( ev_setInitialConditions();)
			initXforms( set, mo );
		st->time = 0.0;
		updateXforms( set, st, mo );

		st->omega = set->omega;
		st->omega2 = set->omega;
		st->omega_p = set->omega_p;
		st->throttleFOM = co->throttle;
		st->a1 = 0;
		st->b1 = 0;
		st->a1fb = 0;
		st->b1fb = 0;
		txy = &( st->tx0 );
		fz = &( st->fz0 );
		for ( j = 0; j < 2 * NGEAR; j++ )
			*txy++ = 0.0;
		for ( j = 0; j < NCONTACTPOINTS; j++ )
			*fz++ = 0.0;

		//mr->vi = 0.0;
		//v->helicopter->tr->vi = 0.0;

		turbGeneratorInit( mo );

		f( v, st ); /* single derivative call */

	}
	else if ( sim.mode == SIM_MODE_RUN ) {

		double newtime;

		newtime = floor( sim.time / set->dtRes ) * set->dtRes;

		/*set->nrepeats = MAX( 1, (int)(sim.dt/set->dtMax) );*/
		set->nrepeats = MAX( 0, ( int ) ( ( newtime - st->time ) / set->dtMax + 0.5 ) );

		if ( set->nrepeats ) {

			//struct controller403_ref *c = &controller403;

			x = &st->time;
			xd = &sd->time;
			in = &st1->time;
			/*st->time = sim.time - sim.dt;*/
			sd->time = 1.0;

			for ( j = 0; j < set->nrepeats; j++ ) {

				/* compute impulse (if any) */

				if ( impulse->on && sim.dt > 0 ) {

					for ( i = 0; i < 3; i++ )
						impulse->f[i] = impulse->impulse[i] / set->dtMax; /*/sim.dt*set->nrepeats;*/

					motion_cross( impulse->r, impulse->f, impulse->m );

					impulse->on = 0;

				}
				else {

					for ( i = 0; i < 3; i++ ) {
						impulse->f[i] = 0;
						impulse->m[i] = 0;
					}

				}

				/* Calculate gusts */
				turbGenerator( set, mo ); /* function assumes update step of set->dtMax */

#if 0
				/* 403 demo controller here (set->dtMax for time step on states) */

				if ( sim.mode == SIM_MODE_INIT ) {
					c->integral_he = 0;
					c->integral_theta = 0;
					c->integral_phi = 0;
				}

				if ( 1 == c->autopilot ) {
					double dx[2], distance, vCmd;

					/* altitude control */
					c->vsCmd = LIMIT( c->Kh * ( c->altitudeCmd - ( -state.p_b_e_L[2] ) ), -c->vsMax, c->vsMax );
					c->integral_he += ( c->vsCmd - ( -state.v_b_e_L[2] ) ) * set->dtMax;
					c->thrust = c->thrustBias + c->Khdot * ( c->vsCmd - ( -state.v_b_e_L[2] ) ) + c->Khi * ( c->integral_he );

					/* position control */
					dx[0] = c->pxCmd - state.p_b_e_L[0];
					dx[1] = c->pyCmd - state.p_b_e_L[1];
					distance = sqrt( SQ( dx[0] ) + SQ( dx[1] ) );
					vCmd = MIN( c->Kpos * distance, c->vCmdMax );
					if ( distance > 0.00001 ) {
						c->vxCmd = vCmd * dx[0] / distance;
						c->vyCmd = vCmd * dx[1] / distance;
					}
					else {
						c->vxCmd = 0;
						c->vyCmd = 0;
					}

					/* velocity control */
					c->integral_theta += ( +cos( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vxCmd - state.v_b_e_L[0] )
																 + sin( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vyCmd - state.v_b_e_L[1] ) ) * set->dtMax;
					c->integral_phi += ( -sin( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vxCmd - state.v_b_e_L[0] )
															 + cos( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vyCmd - state.v_b_e_L[1] ) ) * set->dtMax;

					c->thetaCmd = -c->Kv / 32.174 * C_RAD2DEG * ( +cos( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vxCmd - state.v_b_e_L[0] )
																												+ sin( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vyCmd - state.v_b_e_L[1] ) )
						- c->Kvi / 32.174 * C_RAD2DEG * c->integral_theta;
					c->phiCmd = +c->Kv / 32.174 * C_RAD2DEG * ( -sin( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vxCmd - state.v_b_e_L[0] )
																											+ cos( vehicleOutputs.psi * C_DEG2RAD ) * ( c->vyCmd - state.v_b_e_L[1] ) )
						+ c->Kvi / 32.174 * C_RAD2DEG * c->integral_phi;

					/* attitude control */
					c->roll = c->Kphi * ( c->phiCmd - vehicleOutputs.phi );
					c->pitch = c->Ktheta * ( c->thetaCmd - vehicleOutputs.theta );
					c->yaw = c->Kpsi * hmodDeg( c->psiCmd - vehicleOutputs.psi );
				}
				else {
					c->thrust = -( ( &joyInput[0] )->joyAxis[1] );
					c->roll = ( &joyInput[0] )->joyAxis[2] - c->Kp * state.w_b_e_B[0];
					c->pitch = ( &joyInput[0] )->joyAxis[3] - c->Kq * state.w_b_e_B[1];
					c->yaw = ( &joyInput[0] )->joyAxis[0] - c->Kr * state.w_b_e_B[2];
				}

				c->roll = LIMIT( c->roll, -1, 1 );
				c->pitch = LIMIT( c->pitch, -1, 1 );
				c->yaw = LIMIT( c->yaw, -1, 1 );
				c->thrust = LIMIT( c->thrust, 0, 1 );


				( &siDatalinkMessagePWMToSI )->channel[1] = LIMIT( c->thrust * 1000 - c->roll * 500 + c->pitch * 500 - c->yaw * 500, 0, 1000 );
				( &siDatalinkMessagePWMToSI )->channel[2] = LIMIT( c->thrust * 1000 - c->roll * 500 - c->pitch * 500 + c->yaw * 500, 0, 1000 );
				( &siDatalinkMessagePWMToSI )->channel[3] = LIMIT( c->thrust * 1000 + c->roll * 500 - c->pitch * 500 - c->yaw * 500, 0, 1000 );
				( &siDatalinkMessagePWMToSI )->channel[4] = LIMIT( c->thrust * 1000 + c->roll * 500 + c->pitch * 500 + c->yaw * 500, 0, 1000 );

#endif

				/* integrate - Heun's method (a two stage Runge-Kutta method, using endpoint) */

				f( v, st ); // populates mo->stateDot, among other things. Evaluate derivative at start point - DPM

				for ( i = 0; i < NSTATES; i++ ) { // propagate from left derivative point
					if ( i ) xd[i] *= mo->multiplier;
					in[i] = x[i] + xd[i] * set->dtMax; /*sim.dt/set->nrepeats;*/  //&in[0] is st1
					xd1[i] = xd[i]; /* store derivative */
				}

				normalize_quat( st1->e );
				normalize_quat( st1->e_l );

				/* limit gear deflections */

				txy = &st1->tx0;
				for ( i = 0; i < 2 * NGEAR; i++ ) {
					*txy = MAX( *txy, -pg->txmax );
					*txy = MIN( *txy, pg->txmax );
					txy++;
				}

				f( v, st1 ); //evaluate derivative at endpoint
				for ( i = 0; i < NSTATES; i++ ) {
					if ( i ) xd[i] *= mo->multiplier;
					x[i] += 0.5 * ( xd1[i] + xd[i] ) * set->dtMax; //propagated state uses average of the two derivatives. 
				}

				normalize_quat( st->e );
				normalize_quat( st->e_l );

				/* limit gear deflections */

				txy = &st->tx0;
				for ( i = 0; i < 2 * NGEAR; i++ ) {
					*txy = MAX( *txy, -pg->txmax );
					*txy = MIN( *txy, pg->txmax );
					txy++;
				}

			}

		}

	}

	/* save output data */

	o->model = set->model;
	o->time = sim.time;

	o->phi = mo->phi * C_RAD2DEG;
	o->theta = mo->theta * C_RAD2DEG;
	o->psi = mo->psi * C_RAD2DEG;
	o->thetaFuse = mo->thetaFuse * C_RAD2DEG; // this is freewing fuselage angle

	dcm90[0][0] = -xf->dcm_bl[0][2];
	dcm90[0][1] = xf->dcm_bl[0][1];
	dcm90[0][2] = xf->dcm_bl[0][0];
	dcm90[1][0] = -xf->dcm_bl[1][2];
	dcm90[1][1] = xf->dcm_bl[1][1];
	dcm90[1][2] = xf->dcm_bl[1][0];
	dcm90[2][0] = -xf->dcm_bl[2][2];
	dcm90[2][1] = xf->dcm_bl[2][1];
	dcm90[2][2] = xf->dcm_bl[2][0];
	dcm2euler( dcm90, &o->phi90, &o->theta90, &o->psi90 );
	o->phi90 *= C_RAD2DEG;
	o->theta90 *= C_RAD2DEG;
	o->psi90 *= C_RAD2DEG;

	for ( i = 0; i < 3; i++ ) {
		o->pos[i] = st->p_b_e_L[i];
		o->velocity[i] = xf->v_b_e_B[i];
		o->wind[i] = env->windv[i];
	}

	/*o->throttleLever = co->throttleLever;*/

	/* recomputing here allows definition of alpha to be different in
		 vehicle model */
	if ( mo->tas > mo->vmin ) {
		alpha = atan2( xf->v_b_a_B[2], xf->v_b_a_B[0] );
		beta = atan2( xf->v_b_a_B[1],
									sqrt( SQ( xf->v_b_a_B[0] ) + SQ( xf->v_b_a_B[2] ) ) );
	}
	else {
		alpha = beta = 0.0;
	}

	cal = cos( alpha );
	sal = sin( alpha );
	cbe = cos( beta );
	sbe = sin( beta );

	o->alpha = alpha * C_RAD2DEG;
	o->beta = beta * C_RAD2DEG;
	o->tas = mo->tas * C_FPS2KT;
	o->cas = mo->cas * C_FPS2KT;
	o->latitude = mo->latitude * C_RAD2DEG;
	o->longitude = hmodDeg( mo->longitude * C_RAD2DEG );
	o->altitudeAGL = mo->altitudeAGL - set->zgear;
	o->altitudeMSL = mo->altitudeMSL;

	o->sl_altitudeAGL = mo->sl_altitudeAGL;
	o->sl_altitudeMSL = mo->sl_altitudeMSL;
	o->sl_latitude = mo->sl_latitude * C_RAD2DEG;
	o->sl_longitude = hmodDeg( mo->sl_longitude * C_RAD2DEG );

	o->sl_active = 0;

	o->terrainAlt = o->altitudeMSL - o->altitudeAGL - o->zgear;
	o->sl_terrainAlt = o->sl_altitudeMSL - o->sl_altitudeAGL;

	o->zgear = set->zgear;
	o->vs = mo->vs * 60.0;
	o->gs = mo->gs * C_FPS2KT;
	o->track = mo->track * C_RAD2DEG;
	o->gamma = mo->gamma * C_RAD2DEG;
	o->rpm = st->omega / ( 2.0 * C_PI ) * 60;
	o->G = sqrt( SQ( mo->a[0] ) + SQ( mo->a[1] ) + SQ( mo->a[2] ) ) /
	env->gravity0;
	/*if( mo->a[2] > 0.0 ) o->G = -o->G;*/

	/* landing gear */

	o->wow = mo->wow;

	/* controls */

	o->delm[0] = co->rollStick;
	o->delm[1] = co->pitchStick;
	o->delm[2] = co->rudderPedal;
	o->delf[0] = co->collectiveLever * 2.0 - 1.0;
	o->delt[0] = co->throttleLever * 2.0 - 1.0;

	/* rotors */

	//o->a0 = mr->a0;
	o->a1 = st->a1 * C_RAD2DEG;
	o->b1 = st->b1 * C_RAD2DEG;

	/* get wind axis system */

	xf->dcm_wb[0][0] = cal * cbe;
	xf->dcm_wb[0][1] = -cal * sbe;
	xf->dcm_wb[0][2] = -sal;
	xf->dcm_wb[1][0] = sbe;
	xf->dcm_wb[1][1] = cbe;
	xf->dcm_wb[1][2] = 0.0;
	xf->dcm_wb[2][0] = sal * cbe;
	xf->dcm_wb[2][1] = -sal * sbe;
	xf->dcm_wb[2][2] = cal;
	matrix_multiply( xf->dcm_bl, xf->dcm_wb, xf->dcm_wl );
	matrix_transpose( xf->dcm_wl, xf->dcm_lw );
	dcm2euler( xf->dcm_wl, &o->phiw, &o->thetaw, &o->psiw );

	o->phiw *= C_RAD2DEG;
	o->thetaw *= C_RAD2DEG;
	o->psiw *= C_RAD2DEG;

	dcm90[0][0] = -xf->dcm_wl[0][2];
	dcm90[0][1] = xf->dcm_wl[0][1];
	dcm90[0][2] = xf->dcm_wl[0][0];
	dcm90[1][0] = -xf->dcm_wl[1][2];
	dcm90[1][1] = xf->dcm_wl[1][1];
	dcm90[1][2] = xf->dcm_wl[1][0];
	dcm90[2][0] = -xf->dcm_wl[2][2];
	dcm90[2][1] = xf->dcm_wl[2][1];
	dcm90[2][2] = xf->dcm_wl[2][0];
	dcm2euler( dcm90, &o->phiw90, &o->thetaw90, &o->psiw90 );
	o->phiw90 *= C_RAD2DEG;
	o->thetaw90 *= C_RAD2DEG;
	o->psiw90 *= C_RAD2DEG;

	/* datums */

	o->datumLat = set->refLatitude;
	o->datumLon = set->refLongitude;
	o->datumAlt = set->datumAlt;

	saveFloat( xf->dcm_bl, o->float_dcm_bl );
	saveFloat( xf->dcm_lb, o->float_dcm_lb );
	saveFloat( xf->dcm_wl, o->float_dcm_wl );
	saveFloat( xf->dcm_lw, o->float_dcm_lw );

	saveFloat( xf->sl_dcm_bl, o->sl_float_dcm_bl );
	saveFloat( xf->sl_dcm_lb, o->sl_float_dcm_lb );

}


static void vcross( const double* v1, const double* v2, double* cross ) {

	cross[0] = ( v1[1] * v2[2] ) - ( v1[2] * v2[1] );
	cross[1] = ( v1[2] * v2[0] ) - ( v1[0] * v2[2] );
	cross[2] = ( v1[0] * v2[1] ) - ( v1[1] * v2[0] );

}

static void perturbedHigh( struct vehicle_ref* v, double* Acol, double phi, double theta, double psi ) {

	struct vehicleMotion_ref* mo = v->motion;
	struct motionXforms_ref* x = mo->xforms;
	struct motionLinearize_ref* l = mo->linearize;
	struct state_ref* st = mo->state;
	struct state_ref* sd = mo->stateDot;

	double dvd[3];
	int i;

	f( v, st );

	l->phiDot = st->w_b_e_B[0] + ( st->w_b_e_B[1] * sin( phi ) + st->w_b_e_B[2] * cos( phi ) ) * tan( theta );
	l->thetaDot = st->w_b_e_B[1] * cos( phi ) - st->w_b_e_B[2] * sin( phi );
	l->psiDot = ( st->w_b_e_B[1] * sin( phi ) + st->w_b_e_B[2] * cos( phi ) ) / cos( theta );
	/* get body frame velocity derivative */
	map_vector( x->dcm_lb, sd->v_b_e_L, l->vd_b_e_B );
	map_vector( x->dcm_lb, st->v_b_e_L, l->v_b_e_B );
	vcross( st->w_b_e_B, l->v_b_e_B, dvd );
	for ( i = 0; i < 3; i++ ) l->vd_b_e_B[i] -= dvd[i];   /* is this sign right? */
	Acol[0] = l->phiDot;
	Acol[1] = l->thetaDot;
	Acol[2] = l->psiDot;
	Acol[3] = sd->w_b_e_B[0];
	Acol[4] = sd->w_b_e_B[1];
	Acol[5] = sd->w_b_e_B[2];
	Acol[6] = sd->p_b_e_L[0];
	Acol[7] = sd->p_b_e_L[1];
	Acol[8] = sd->p_b_e_L[2];
	Acol[9] = l->vd_b_e_B[0];
	Acol[10] = l->vd_b_e_B[1];
	Acol[11] = l->vd_b_e_B[2];
	Acol[12] = sd->omega;
	Acol[13] = sd->a1;
	Acol[14] = sd->b1;
	Acol[15] = sd->a1fb;
	Acol[16] = sd->b1fb;
	Acol[17] = sd->thetaFuse;
	Acol[18] = sd->thetaFusedot;

}

static void perturbedLow( struct vehicle_ref* v, double* Acol, double phi, double theta, double psi, double perturbation ) {

	struct vehicleMotion_ref* mo = v->motion;
	struct motionXforms_ref* x = mo->xforms;
	struct motionLinearize_ref* l = mo->linearize;
	struct state_ref* st = mo->state;
	struct state_ref* sd = mo->stateDot;

	double dvd[3];
	int i;

	f( v, st );

	l->phiDot = st->w_b_e_B[0] + ( st->w_b_e_B[1] * sin( phi ) + st->w_b_e_B[2] * cos( phi ) ) * tan( theta );
	l->thetaDot = st->w_b_e_B[1] * cos( phi ) - st->w_b_e_B[2] * sin( phi );
	l->psiDot = ( st->w_b_e_B[1] * sin( phi ) + st->w_b_e_B[2] * cos( phi ) ) / cos( theta );
	/* get body frame velocity derivative */
	map_vector( x->dcm_lb, sd->v_b_e_L, l->vd_b_e_B );
	map_vector( x->dcm_lb, st->v_b_e_L, l->v_b_e_B );
	vcross( st->w_b_e_B, l->v_b_e_B, dvd );
	for ( i = 0; i < 3; i++ ) l->vd_b_e_B[i] -= dvd[i];   /* is this sign right? */
	Acol[0] = ( Acol[0] - l->phiDot ) / ( perturbation * 2 );
	Acol[1] = ( Acol[1] - l->thetaDot ) / ( perturbation * 2 );
	Acol[2] = ( Acol[2] - l->psiDot ) / ( perturbation * 2 );
	Acol[3] = ( Acol[3] - sd->w_b_e_B[0] ) / ( perturbation * 2 );
	Acol[4] = ( Acol[4] - sd->w_b_e_B[1] ) / ( perturbation * 2 );
	Acol[5] = ( Acol[5] - sd->w_b_e_B[2] ) / ( perturbation * 2 );
	Acol[6] = ( Acol[6] - sd->p_b_e_L[0] ) / ( perturbation * 2 );
	Acol[7] = ( Acol[7] - sd->p_b_e_L[1] ) / ( perturbation * 2 );
	Acol[8] = ( Acol[8] - sd->p_b_e_L[2] ) / ( perturbation * 2 );
	Acol[9] = ( Acol[9] - l->vd_b_e_B[0] ) / ( perturbation * 2 );
	Acol[10] = ( Acol[10] - l->vd_b_e_B[1] ) / ( perturbation * 2 );
	Acol[11] = ( Acol[11] - l->vd_b_e_B[2] ) / ( perturbation * 2 );
	Acol[12] = ( Acol[12] - sd->omega ) / ( perturbation * 2 );
	Acol[13] = ( Acol[13] - sd->a1 ) / ( perturbation * 2 );
	Acol[14] = ( Acol[14] - sd->b1 ) / ( perturbation * 2 );
	Acol[15] = ( Acol[15] - sd->a1fb ) / ( perturbation * 2 );
	Acol[16] = ( Acol[16] - sd->b1fb ) / ( perturbation * 2 );
	Acol[17] = ( Acol[17] - sd->thetaFuse ) / ( perturbation * 2 );
	Acol[18] = ( Acol[18] - sd->thetaFusedot ) / ( perturbation * 2 );

}

void modelmultirotorCmd( int argc, char** argv ) {

	struct vehicleSet_ref* set = &vehicleSet;
	struct vehicleMotion_ref* mo = &vehicleMotion;
	struct mass_ref* m = mo->mass;
	struct params_gear_ref* g = mo->gear;
	struct vehicleOutputs_ref* no = &gcs0Outputs;
	struct motionControls_ref* co = mo->controls;
	struct si_ref* s = &si;
	struct navigation_ref* nav = &navigation;

	struct vehicle_ref* veh = &vehicle;
	struct multirotor_ref* multirotor = veh->multirotor;
	struct rotorMultirotor_ref* ro = multirotor->ro;  //ro: rotor

	int i;
	double legRadius;
	double legz;

	set->model = MODEL_MULTIROTOR;
	no->model = MODEL_MULTIROTOR;

	// From Eric's textbook
	m->mass = 0.0870267; // 2.8 lb
	m->Ixx = 0.032; // 8 motors at 0.001919 slugs each, main mass at 0.8549177 slugs estimated at 1/2 of mass 4.5 inches from center on either side
	m->Iyy = 0.032; // 8 motors at 0.001919 slugs each, main mass at 0.8549177 slugs estimated at 1/2 of mass 3.5 inches from center on either side
	m->Izz = 0.052; // 8 motors at 0.001919 slugs each, main mass at 0.8549177 slugs estimated centered in a ring 4 inches from center
	m->Ixz = 0.0;

	set->zgear = 0.37;

	//co->mode = MOTIONINPUT_JOYSTICK; // default is already using a joystick
	co->joystickMode = JOYSTICK_MODE_FLYSKY;
	co->setInputTypes = 1; // Set the GCS input settings

	/* gear */
	g->frcoeff = 1.0;
	legRadius = 0.3 / cos( 45 * C_DEG2RAD );
	legz = set->zgear + 0.1;
	/*to make this work zw has to be greater than zgear. Initialize such that
	the gear is under the ground!! then let gear dynamics "pop it up" */
	g->xw[4] = legRadius * cos( 90 * C_DEG2RAD );
	g->yw[4] = legRadius * sin( 90 * C_DEG2RAD );
	g->zw[4] = legz;
	g->xw[5] = legRadius * cos( 270 * C_DEG2RAD );
	g->yw[5] = legRadius * sin( 270 * C_DEG2RAD );
	g->zw[5] = legz;
	g->xw[7] = legRadius * cos( 30 * C_DEG2RAD );
	g->yw[7] = legRadius * sin( 30 * C_DEG2RAD );
	g->zw[7] = legz;
	g->xw[8] = legRadius * cos( 150 * C_DEG2RAD );
	g->yw[8] = legRadius * sin( 150 * C_DEG2RAD );
	g->zw[8] = legz;
	g->xw[9] = legRadius * cos( -30 * C_DEG2RAD );
	g->yw[9] = legRadius * sin( -30 * C_DEG2RAD );
	g->zw[9] = legz;
	g->xw[10] = legRadius * cos( -150 * C_DEG2RAD );
	g->yw[10] = legRadius * sin( -150 * C_DEG2RAD );
	g->zw[10] = legz;

	g->xw[6] = 0;
	g->yw[6] = 0;
	g->zw[6] = -1;

	g->kg[4] = 28.0 / 6 / 0.1;
	g->kg[5] = 28.0 / 6 / 0.1;
	g->kg[7] = 28.0 / 6 / 0.1;
	g->kg[8] = 28.0 / 6 / 0.1;
	g->kg[9] = 28.0 / 6 / 0.1;
	g->kg[10] = 28.0 / 6 / 0.1;

	g->kg[6] = 28.0 / 0.1;
	for ( i = 0; i < 4; i++ ) {
		g->xw[i] = 0;
		g->yw[i] = 0;
		g->zw[i] = 0;
		g->kg[i] = 1;
	}
	g->Kx = 0.5;
	g->Ky = 0.5;

	//sonar
	s->sonar->encodeMode = 0; //1=quadrotor 0=rmax
	s->sonar->r[0] = 0.0;
	s->sonar->r[1] = 0.0;
	s->sonar->r[2] = 0.1;

	s->sonar->alt_min = 0.1;
	s->sonar->alt_max = 20.0;
	s->sonar->sf = 1;
	s->sonar->outlierEnable = 1;
	s->sonar->chance = 15;

	logInfo( "flying multirotor" );
}

void initMotion( void ) {

	struct vehicleMotion_ref* mo = &vehicleMotion;
	struct state_ref* st = mo->state;
	struct motionXforms_ref* xf = mo->xforms;
	/*struct trimmer_ref *trimmer = NULL;*/

	build_rotmatrix( xf->dcm_bl, st->e );
	matrix_transpose( xf->dcm_bl, xf->dcm_lb );

	updateMotion();
	commandLoad( "multirotor", modelmultirotorCmd, "select multirotor model" );
}



