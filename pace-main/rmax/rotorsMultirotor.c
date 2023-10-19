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
 **
 * written by Ep Pravitra jpravitra@gatech.edu
 * Modified by Joel Dunham jdunham@adaptiveflight.com
 *
 * Last version: Sept 2013
 */
#include <math.h>

#include "esim/util.h"
#include "esim/quat.h"
#include "rmax/matrix.h"
#include "rmax/motion_ref.h"
#include "rmax/onboard_ref.h"
#include "stdio.h" 
/******************************************************************************
 */

void eachRotorMulti( struct vehicleMotion_ref *mo,struct state_ref *s, int rn ) { //rn: rotor number

    struct vehicle_ref        *veh       = &vehicle;
    struct multirotor_ref     *multirotor = veh->multirotor;
    struct env_ref            *env       = mo->env;
    struct rotorMultirotor_ref *ro       = multirotor->ro;  //ro: rotor
    struct motionXforms_ref   *x         = mo->xforms;

	double v_p_a_B[3], v_p_a_R[3];
    int i;

	/* velocity of prop wrt air, body frame */
	for( i=0; i<3; i++ ) v_p_a_B[i] = x->v_b_a_B[i];
	v_p_a_B[0] += ro->z[rn]*x->w_b_a_B[1] - ro->y[rn]*x->w_b_a_B[2];
	v_p_a_B[1] += ro->x[rn]*x->w_b_a_B[2] - ro->z[rn]*x->w_b_a_B[0];
	v_p_a_B[2] += ro->y[rn]*x->w_b_a_B[0] - ro->x[rn]*x->w_b_a_B[1];

	/* now wrt propeller */
    map_T_vector( ro->dcm_rb[rn], v_p_a_B, v_p_a_R );

    ro->fr = ro->cd0*ro->r*ro->b*ro->c;
    ro->wb[rn] = v_p_a_R[2] + 0.6667*ro->omega[rn]*ro->r*( 0.75*ro->twist );

    /* ground effect */
    ro->h[rn] = -s->p_b_e_L[2] - ( ro->x[rn]*x->dcm_bl[0][2] + ro->y[rn]*x->dcm_bl[1][2] + ro->z[rn]*x->dcm_bl[2][2] ) - env->terrainAlt; // height above ground
    //ro->h[rn]=1; (for testing)
    
    if( ro->h[rn] > 0.01 )
        ro->ge_factor[rn] = 1.0/( 1.0 + ro->ground_effect*SQ( 2.0*ro->r/ro->h[rn]) );
    else
        ro->ge_factor[rn] = 0.0;
    
    /* solve for thrust */
    ro->solve_used[rn] = 0;
    do {
        ro->thrust[rn] = ( ro->wb[rn] - ro->vi[rn] )*
            ro->omega[rn]*ro->r*ro->r*env->rho*ro->a*ro->b*ro->c*0.25;

        ro->vprime[rn] = sqrt( v_p_a_R[0]*v_p_a_R[0] + v_p_a_R[1]*v_p_a_R[1] +
		           SQ( v_p_a_R[2] - ro->vi[rn] ) );

        if( ABS( ro->vprime[rn] ) > 0.001 ) {//creates a bug if the initial velocity is zero
            ro->new_vi[rn] = ro->ge_factor[rn]*ro->thrust[rn]*0.5/
            ( env->rho*C_PI*ro->r*ro->r*ro->vprime[rn] );
        } else {
            ro->new_vi[rn] = 0.0;  
        }
        
        ro->correction[rn] = ro->solve_gain*( ro->new_vi[rn] - ro->vi[rn] );
        ro->vi[rn] += ro->correction[rn];
        ro->solve_used[rn]++;

    } while( ( ABS( ro->correction[rn] ) > ro->solve_tol ) &&
        ( ro->solve_used[rn] < ro->solve_steps ) );

    /* solve for rotor hub moment */

	ro->power[rn] = ro->thrust[rn]*( ro->vi[rn] - v_p_a_R[2] ) + 0.5*env->rho*ro->fr*0.25*ro->omega[rn]*ro->r*
      ( ro->omega[rn]*ro->omega[rn]*ro->r*ro->r + ( v_p_a_R[0]*v_p_a_R[0] + v_p_a_R[1]*v_p_a_R[1] ) );

	if( ro->omega[rn] > 0.01 ) 
		ro->torqueR[rn] = ro->power[rn]/ro->omega[rn];
	else
		ro->torqueR[rn] = 0.0;

	ro->torqueE[rn] = ro->throttle[rn]*ro->torqueMax;

	/*if( ro->throttle[rn] > 0.14 && ro->torqueE[rn] < ro->torqueR[rn] ) {
		ro->torqueE[rn] += ( ro->torqueE[rn] - ro->torqueR[rn] )*10;
		ro->torqueE[rn] = LIMIT( ro->torqueE[rn], -ro->torqueMax, ro->torqueMax );
	}*/

    ro->omegad[rn] = LIMIT( ( ro->torqueE[rn] - ro->torqueR[rn] )/ro->i_r, -ro->omegadLimit, ro->omegadLimit );

    /* First order model for relating throttle to RPM directly 2*C_PI/60 is from rpm to rps*/
	//ro->omegad[rn] = ((2*C_PI/60)*ro->max_rpm* ro->throttle[rn] - ro->omega[rn])/( ro->rotor_tc );
	//ro->torqueE[rn] += ro->omegad[rn]*ro->i_r;

    /* still need to add */
    // 1.roll moment due to forward flight (quadrotor has no hinge)
    ro->rollMoment[rn] = 0.0;

    // 2.hub force
	ro->hubForceX[rn] = -env->rho*ro->cd0*ro->b*ro->c*ro->omega[rn]*SQ( ro->r )*0.25*v_p_a_R[0];
    ro->hubForceY[rn] = -env->rho*ro->cd0*ro->b*ro->c*ro->omega[rn]*SQ( ro->r )*0.25*v_p_a_R[1];

}

void rotorsMultirotor( struct vehicleMotion_ref *mo, struct state_ref *s ) {

    struct vehicle_ref         *veh        = &vehicle;
    struct multirotor_ref      *multirotor = veh->multirotor;
    struct rotorMultirotor_ref *ro         = multirotor->ro;  //ro: rotor

	struct onboard_ref         *ob         = &onboard;
	struct actuatorInt_ref     *act        = ob->actuators;
	struct pwmMultirotor_ref   *multi      = act->pwmMulti;

	double force[3], arm[3], moment[3];
    double power = 0.0;
	int i;

	for( i = 0; i < ro->numRotors; ++i ) {
		eachRotorMulti( mo, s, i );
        power += ro->power[i];
	}

    ro->totalPower = power;
    /* zero forces and moments */
    ro->rotorx = 0.0;
    ro->rotory = 0.0;
    ro->rotorz = 0.0;
    ro->rotorl = 0.0;
    ro->rotorm = 0.0;
    ro->rotorn = 0.0;
    
	for(i = 0; i < ro->numRotors; ++i ) {

		/* model poor prop */
		ro->thrust[i] *= ro->propDamage[i];

		/* thrust force */
		force[0] = -ro->thrust[i]*ro->dcm_rb[i][0][2];
		force[1] = -ro->thrust[i]*ro->dcm_rb[i][1][2];
		force[2] = -ro->thrust[i]*ro->dcm_rb[i][2][2];

		/* hub force */
		force[0] += ro->hubForceX[i]*ro->dcm_rb[i][0][0];
		force[1] += ro->hubForceX[i]*ro->dcm_rb[i][1][0];
		force[2] += ro->hubForceX[i]*ro->dcm_rb[i][2][0];
		force[0] += ro->hubForceY[i]*ro->dcm_rb[i][0][1];
		force[1] += ro->hubForceY[i]*ro->dcm_rb[i][1][1];
		force[2] += ro->hubForceY[i]*ro->dcm_rb[i][2][1];

		ro->rotorx += force[0];
		ro->rotory += force[1];
		ro->rotorz += force[2];

		/* motor torque */
		ro->rotorl += -ro->torqueE[i]*multi->direction[i]*ro->propDamage[i]*ro->dcm_rb[i][0][2];
		ro->rotorm += -ro->torqueE[i]*multi->direction[i]*ro->propDamage[i]*ro->dcm_rb[i][1][2];
		ro->rotorn += -ro->torqueE[i]*multi->direction[i]*ro->propDamage[i]*ro->dcm_rb[i][2][2];

		/* moment from thrust (this is all in the body frame now) */
		arm[0] = ro->x[i];  arm[1] = ro->y[i];  arm[2] = ro->z[i];
		mat_cross( arm, force, moment );
		ro->rotorl += moment[0];
		ro->rotorm += moment[1];
		ro->rotorn += moment[2];

		/* rolling moment due to forward flight */
		// ro->rotorl += ...
		// ro->rotorm += ...
		// ro->rotorn += ...
	}
}


