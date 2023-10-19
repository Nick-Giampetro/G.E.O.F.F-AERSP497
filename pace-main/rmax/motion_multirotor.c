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
 *
 * Last version: Mar 2010
 ***/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esim/util.h"
#include "esim/quat.h"
#include "esim/sim_ref.h"
#include "esim/rand.h"

#include "rmax/motion.h"
#include "rmax/motion_ref.h"
#include "rmax/xforms.h"
#include "rmax/motion_multirotor.h"
#include "rmax/gear_ref.h"
#include "rmax/gear.h"
#include "rmax/aeroMultirotor.h"
#include "rmax/rotorsMultirotor.h"
#include "rmax/matrix.h"
#include "rmax/si_ref.h"
#include "rmax/onboard_ref.h"

void multirotor_f( struct vehicleSet_ref *set,struct vehicleMotion_ref *mo, struct state_ref *s, double dt ) {

	struct vehicle_ref        *veh        = &vehicle;
	struct multirotor_ref      *multirotor = veh->multirotor;
	struct mass_ref           *m          = mo->mass;
	struct motionXforms_ref   *x          = mo->xforms;
	struct aeroMultirotor_ref  *a         = multirotor->aero;
	struct rotorMultirotor_ref *ro        = multirotor->ro;  
	struct params_gear_ref    *g          = mo->gear;
	struct env_ref            *env        = mo->env;
    struct si_ref             *simin     = &si;
	struct state_ref          *sd        = mo->stateDot; /*state derivative*/
	
	/* this stuff should be moved to si.c */
	struct datalinkMessagePWM_ref *pwm   = simin->act->pwmToSI;
    struct onboard_ref            *ob    = &onboard;
	struct pwmMultirotor_ref      *multi = ob->actuators->pwmMulti;

    int    i, j;
    
    double gamma, pwmDesired;
	double a_b_e_B[3];
	double propz, dx[3];
	char armed;

    // to make reinitialization work
    // may have to be moved later
    if( sim.mode == SIM_MODE_INIT ) {

		for( i=0; i<ro->numRotors; i++ ) {
          s->multirotorRotorRpm[i]   = 0;
          s->multirotorThrottleState[i] = 0;
        }
		ro->motorsArmed = 0;

		// Calculate the location of the motors
		ro->numRotors = multi->numberOfRotors;
		for( i=0; i<ro->numRotors; ++i ) {

			/* determine phase angle */
            if( multi->rotorsPaired ) {
                ro->phase[i] = C_PI*2.0*( 0.5 + i )/(0.5*multi->numberOfRotors) + multi->phaseAngle*C_DEG2RAD; /* 0 out the nose, 90deg on right wing */
            } else {
                ro->phase[i] = C_PI*2.0*( 0.5 + i )/(    multi->numberOfRotors) + multi->phaseAngle*C_DEG2RAD; /* 0 out the nose, 90deg on right wing */
            }

			/* find dcm for each rotor */
			/* note chose benefitial yaw authority choice for motor twists */

            if(!multirotor->ro->twistOff){
                ro->twistPer[i] = -ro->motorTwist*multi->direction[i];
            }else{
                 /* add ability to offset the twist of each rotor by a little bit to sim it not being perfect */
                if(!multirotor->ro->tmp){ //tmp makes this only happen once - if not it keeps resetting during init which bricks it
                    ro->twistPer[i] = -ro->motorTwist*multi->direction[i] + multirotor->ro->twist_offset*C_DEG2RAD*randne();
                }

            }

			/* find dcm for each rotor */
			/* note chose benefitial yaw authority choice for motor twists */
			euler2dcm( ro->twistPer[i], ro->dihedral, ro->phase[i], ro->dcm_rb[i] );

			// Calculate the x/y location
			if( ro->phase[i] < C_PI*2 ) { /* paired props go other way */
				propz = -ro->propz;
			} else {
				propz = +ro->propz;
			}
			for( j=0; j<3; j++ ) {
				dx[j] = ro->dcm_rb[i][j][2]*propz;
			}
			ro->x[i] = dx[0] + ro->armRadius*cos( ro->phase[i] )*cos( ro->dihedral );
			ro->y[i] = dx[1] + ro->armRadius*sin( ro->phase[i] )*cos( ro->dihedral );
			ro->z[i] = dx[2] + ro->armz - ro->armRadius*sin( ro->dihedral );

		}
        if(multirotor->ro->twistOff){ //if you use the reset button in graphics this does not get reset--might want to fix this
            multirotor->ro->tmp = 1; //used to make twist off set calc only happen once
        }
    }
    
	for( i=0; i<ro->numRotors; i++ ) {
		ro->omega[i] = s->multirotorRotorRpm[i];
	}

	/* zero forces and moments */

	for( i=0; i<3; i++ ) {
		mo->f[i] = mo->fe_B[i];
		mo->m[i] = mo->me_B[i];
	}

	/* Update transforms, velocities, etc. */

	env->turbulence->span = 3.0; /* should link this to a variable, distance between props? */
	env->windv[0] = set->windss[0] + env->turbulence->gust_v[0];
	env->windv[1] = set->windss[1] + env->turbulence->gust_v[1];
	env->windv[2] = set->windss[2] + env->turbulence->gust_v[2];
	env->windw[0] = env->turbulence->gust_w[0];
	env->windw[1] = env->turbulence->gust_w[1];
	env->windw[2] = env->turbulence->gust_w[2];

	updateXforms( set, s, mo );

	/* air data */

	mo->vtot = sqrt( SQ( x->v_b_e_B[0] ) + SQ( x->v_b_e_B[1] ) +
		SQ( x->v_b_e_B[2] ) );
	mo->tas = sqrt( SQ( x->v_b_a_B[0] ) + SQ( x->v_b_a_B[1] ) +
		SQ( x->v_b_a_B[2] ) );
	mo->cas = sqrt( env->densityRatio )*mo->tas;
	/*mo->mach = mo->tas/(760.9*5280.0/3600.0);  approx */

	if( mo->tas > mo->vmin ) {
		mo->alpha = atan2( x->v_b_a_B[2], x->v_b_a_B[0] );
		mo->beta  = atan2( x->v_b_a_B[1],
			sqrt( SQ( x->v_b_a_B[0] ) + SQ( x->v_b_a_B[2] ) ) );
		mo->taseff = mo->tas;
	} else {
		mo->alpha  = mo->beta = 0.0;
		mo->taseff = mo->vmin;
	}

	mo->cal = cos( mo->alpha );
	mo->sal = sin( mo->alpha );
	mo->cbe = cos( mo->beta );
	mo->sbe = sin( mo->beta );
    
	/* Map control inputs */
	// TODO_JPD: is this conversion correct?
	if( ro->motorsArmed == 1 ) {
		for(i=0; i<ro->numRotors; ++i) {
			/* The true PWM range is 0 - 999, not the artificially narrowed range that may be used in the controller. */
			pwmDesired = (double)pwm->channel[multi->order[i]-1] / (double)(999.0 - 0.0);

			/* rate limit */
			ro->pwm[multi->order[i]-1] = s->multirotorThrottleState[multi->order[i]-1];
			sd->multirotorThrottleState[multi->order[i]-1] = LIMIT( ( pwmDesired - ro->pwm[multi->order[i]-1] )/dt, -ro->pwmRateLimitDown, ro->pwmRateLimitUp );

			/* nonlinear map of PWM to torque */
			ro->throttle[multi->order[i]-1] = sqrt( LIMIT(( ro->pwm[multi->order[i]-1] - ro->pwmBreakout )/( 1.0 - ro->pwmBreakout ), 0, 1 ) );
		}
	} else {
		armed = 1;
		for(i=0; i<ro->numRotors; ++i) {
			ro->pwm[multi->order[i]-1] = 0.0;
			ro->throttle[multi->order[i]-1] = 0.0;
			sd->multirotorThrottleState[multi->order[i]-1] = LIMIT( -s->multirotorThrottleState[multi->order[i]-1]/dt, -ro->pwmRateLimitDown, ro->pwmRateLimitUp );
			if( pwm->channel[multi->order[i]] > 1 ) {
				armed = 0;
			}
		}
		if( armed == 1 ) {
			ro->motorsArmed = 1;
		}
	}

    /* add aero forces and moments */
	aerodynamicsMultirotor( multirotor->aero, mo );
	mo->f[0] += a->aerox;
	mo->f[1] += a->aeroy;
	mo->f[2] += a->aeroz;
	mo->m[0] += a->aerol;
	mo->m[1] += a->aerom;
	mo->m[2] += a->aeron;
    
    /* add rotor forces and moments */
    rotorsMultirotor( mo, s );
    mo->f[0] += ro->rotorx;
	mo->f[1] += ro->rotory;
	mo->f[2] += ro->rotorz;
	mo->m[0] += ro->rotorl;
	mo->m[1] += ro->rotorm;
	mo->m[2] += ro->rotorn;

	/* landing gear/skids */
	gear( env->terrainAlt, mo, s, sd, dt );
	mo->f[0] += g->gearx;
	mo->f[1] += g->geary;
	mo->f[2] += g->gearz;
	mo->m[0] += g->gearl;
	mo->m[1] += g->gearm;
	mo->m[2] += g->gearn;

    /* moments due to gyroscopic effect */
	for( j=0; j<3; j++ ) ro->angMomentum[j] = 0;
	for( i=0; i<ro->numRotors; i++ ) {
		for( j=0; j<3; j++ ) {
			ro->angMomentum[j] += ro->i_r*ro->omega[i]*multi->direction[i]*ro->dcm_rb[i][j][2];
		}
	}
	mat_cross( s->w_b_e_B, ro->angMomentum, ro->gyroMoment );
	for( j=0; j<3; j++ ) { 
		mo->m[j] -= ro->gyroMoment[j];
	}

    /* get gravity force in body frame */
    for( i=0; i<3; i++ )
        mo->f[i] += m->mass*x->dcm_lb[i][2]*env->gravity;

    /* impulse (if any) */
    for( i=0; i<3; i++ ) {
        mo->f[i] += mo->impulse->f[i];
		mo->m[i] += mo->impulse->m[i];
	}

    /* position update */
    for( i=0; i<3; i++ )
        sd->p_b_e_L[i] = s->v_b_e_L[i];

	/* state derivatives, velicity */
	for( i=0; i<3; i++ )
		a_b_e_B[i] = mo->f[i]/m->mass * (mo->linaccflag);//*0;

	map_vector( x->dcm_bl, a_b_e_B, sd->v_b_e_L );

	/* acceleration outputs (accelerometers at c.g.) */
	for( i=0; i<3; i++ )
		mo->a[i] = a_b_e_B[i] - x->dcm_lb[i][2]*env->gravity;

	/* angular rate update */
	gamma = m->Ixx*m->Izz - SQ( m->Ixz );

	sd->w_b_e_B[0] = ( m->Izz*mo->m[0] + m->Ixz*mo->m[2] +
		s->w_b_e_B[1]*s->w_b_e_B[2]*
		( m->Izz*( m->Iyy - m->Izz ) - SQ( m->Ixz ) ) +
		s->w_b_e_B[0]*s->w_b_e_B[1]*
		m->Ixz*( m->Ixx - m->Iyy + m->Izz )
		)/gamma;
	sd->w_b_e_B[1] = ( mo->m[1] + /* mo->Mad*mo->alphadot + */
		s->w_b_e_B[2]*s->w_b_e_B[0]*( m->Izz - m->Ixx ) +
		( SQ( s->w_b_e_B[2] ) - SQ( s->w_b_e_B[0] ) )*m->Ixz
		)/m->Iyy;
	sd->w_b_e_B[2] = ( m->Ixz*mo->m[0] + m->Ixx*mo->m[2] +
		s->w_b_e_B[0]*s->w_b_e_B[1]*
		( m->Ixx*( m->Ixx - m->Iyy ) + SQ( m->Ixz ) ) +
		s->w_b_e_B[1]*s->w_b_e_B[2]*
		m->Ixz*( m->Iyy - m->Izz - m->Ixx )
		)/gamma;

	for(i=0;i<3;i++)
		sd->w_b_e_B[i] *= mo->angaccflag;

	/* attitude update */
	omega2edot( s->w_b_e_B[0], s->w_b_e_B[1], s->w_b_e_B[2], s->e, sd->e );

    /* high bandwidth outputs for other vehicle modules */

	/* flight path outputs */

	mo->gs = sqrt( SQ( s->v_b_e_L[0] ) + SQ( s->v_b_e_L[1] ) );
	if( mo->gs > 1.0 ) {
		mo->track = atan2( s->v_b_e_L[1], s->v_b_e_L[0] );
		mo->gamma = -atan2( s->v_b_e_L[2], mo->gs );
	} else {
		mo->track = mo->psi;
		mo->gamma = mo->theta;
	}
	mo->vs = -s->v_b_e_L[2];

    /* integration for throttle state and rotor angular velocities */

	for(i=0; i<ro->numRotors; i++) {
		sd->multirotorRotorRpm[i] = ro->omegad[i];
	}

}
