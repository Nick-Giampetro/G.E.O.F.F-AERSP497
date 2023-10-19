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
#include "esim/util.h"
#include "esim/quat.h"
#include "rmax/motion_ref.h"
#include "rmax/xforms.h"


void initXforms( struct vehicleSet_ref *set,
		 struct vehicleMotion_ref *mo ) {

  struct motionXforms_ref *x = mo->xforms;
  struct state_ref *st  = mo->state;
  double chphi, shphi, chthe, shthe, chpsi, shpsi;
  double chphi_l, shphi_l, chthe_l, shthe_l, chpsi_l, shpsi_l;

  /* set position */

  st->p_b_e_L[0] =  set->xNorth;
  st->p_b_e_L[1] =  set->xEast;
  st->p_b_e_L[2] = -set->altitude - set->zgear - mo->env->terrainAlt;

  /* set attitude */

  shphi = sin( 0.5*set->phi*C_DEG2RAD );
  chphi = cos( 0.5*set->phi*C_DEG2RAD );
  shthe = sin( 0.5*set->theta*C_DEG2RAD );
  chthe = cos( 0.5*set->theta*C_DEG2RAD );
  shpsi = sin( 0.5*set->psi*C_DEG2RAD );
  chpsi = cos( 0.5*set->psi*C_DEG2RAD );
  st->e[0] = chpsi*chthe*chphi + shpsi*shthe*shphi;
  st->e[1] = chpsi*chthe*shphi - shpsi*shthe*chphi;
  st->e[2] = chpsi*shthe*chphi + shpsi*chthe*shphi;
  st->e[3] = shpsi*chthe*chphi - chpsi*shthe*shphi;

  if(mo->isFreewing == 0 ){
  st->e_l[0] = 1;
  st->e_l[1] = 0;
  st->e_l[2] = 0;
  st->e_l[3] = 0;}else{
  shphi_l = sin( 0.5*set->phi_l*C_DEG2RAD );
  chphi_l = cos( 0.5*set->phi_l*C_DEG2RAD );
  shthe_l = sin( 0.5*set->theta_l*C_DEG2RAD );
  chthe_l = cos( 0.5*set->theta_l*C_DEG2RAD );
  shpsi_l = sin( 0.5*set->psi_l*C_DEG2RAD );
  chpsi_l = cos( 0.5*set->psi_l*C_DEG2RAD );
  st->e_l[0] = chpsi_l*chthe_l*chphi_l + shpsi_l*shthe_l*shphi_l;
  st->e_l[1] = chpsi_l*chthe_l*shphi_l - shpsi_l*shthe_l*chphi_l;
  st->e_l[2] = chpsi_l*shthe_l*chphi_l + shpsi_l*chthe_l*shphi_l;
  st->e_l[3] = shpsi_l*chthe_l*chphi_l - chpsi_l*shthe_l*shphi_l;
  }

  /* set velocity */

  x->v_b_e_B[0] = set->u;
  x->v_b_e_B[1] = set->v;
  x->v_b_e_B[2] = set->w;
  map_vector( x->dcm_bl, x->v_b_e_B, st->v_b_e_L );


  st->v_l_e_L[0] = 0;
  st->v_l_e_L[1] = 0;
  st->v_l_e_L[2] = 0;

  /* angular rate */

  st->w_b_e_B[0] = set->p*C_DEG2RAD;
  st->w_b_e_B[1] = set->q*C_DEG2RAD;
  st->w_b_e_B[2] = set->r*C_DEG2RAD;

  st->w_l_e_B[0] = 0;
  st->w_l_e_B[1] = 0;
  st->w_l_e_B[2] = 0;

  normalize_quat( st->e );
  normalize_quat( st->e_l );
  
  /* set freewing */
  st->thetaFuse = set->thetaFuse*C_DEG2RAD; //- set->theta*C_DEG2RAD;
  st->thetaFusedot = set->thetaFusedot*C_DEG2RAD;

}

void updateXforms( struct vehicleSet_ref *set,
		   struct state_ref *s,
		   struct vehicleMotion_ref *mo ) {

  struct motionXforms_ref *x = mo->xforms;
  struct env_ref        *env = mo->env;

  double v_a_e_B[3], w_a_e_B[3];

  /* position */

  mo->latitude  = ( set->refLatitude  + s->p_b_e_L[0]/(60*C_NM2FT) )*C_DEG2RAD;
  mo->longitude = hmodDeg( set->refLongitude + s->p_b_e_L[1]/(60*C_NM2FT)
		    /cos( set->refLatitude*C_DEG2RAD ) )*C_DEG2RAD;
  mo->altitudeAGL = -s->p_b_e_L[2] - mo->env->terrainAlt;
  mo->altitudeMSL = -s->p_b_e_L[2] + set->datumAlt;

  
#ifndef BUILD_RAFSA_LIBRARY
	/* slungload */
  mo->sl_latitude  = ( set->refLatitude  + s->p_l_e_L[0]/(60*C_NM2FT) )*C_DEG2RAD;
  mo->sl_longitude = hmodDeg( set->refLongitude + s->p_l_e_L[1]/(60*C_NM2FT)
		    /cos( set->refLatitude*C_DEG2RAD ) )*C_DEG2RAD;
  mo->sl_altitudeAGL = -s->p_l_e_L[2] - mo->env->terrainAlt;
  mo->sl_altitudeMSL = -s->p_l_e_L[2] + set->datumAlt;
#endif
  
  /* get frame transformation matricies */

  build_rotmatrix( x->dcm_bl, s->e );
  matrix_transpose( x->dcm_bl, x->dcm_lb );

  build_rotmatrix( x->sl_dcm_bl, s->e_l );
  matrix_transpose( x->sl_dcm_bl, x->sl_dcm_lb );


  /* get euler angles from body to local transformation */

  dcm2euler( x->dcm_bl, &mo->phi, &mo->theta, &mo->psi );
  mo->sphi = sin( mo->phi );
  mo->cphi = cos( mo->phi );
  mo->stheta = sin( mo->theta );
  mo->ctheta = cos( mo->theta );

  /* get velocities */

  map_vector( x->dcm_lb, s->v_b_e_L, x->v_b_e_B );

  map_vector( x->dcm_lb, env->windv, v_a_e_B );
  x->v_b_a_B[0] = x->v_b_e_B[0] - v_a_e_B[0];
  x->v_b_a_B[1] = x->v_b_e_B[1] - v_a_e_B[1];
  x->v_b_a_B[2] = x->v_b_e_B[2] - v_a_e_B[2];

  /* get angular rates */

  map_vector( x->dcm_bl, s->w_b_e_B, x->w_b_e_L );

  map_vector( x->dcm_lb, env->windw, w_a_e_B );
  x->w_b_a_B[0] = s->w_b_e_B[0] - w_a_e_B[0];
  x->w_b_a_B[1] = s->w_b_e_B[1] - w_a_e_B[1];
  x->w_b_a_B[2] = s->w_b_e_B[2] - w_a_e_B[2];

	/* atmosphere model */

    /* 1976 Standard Atmosphere up to the tropopause (about 36000 feet) */

	env->temperature = env->T0 + env->lapseRate*mo->altitudeMSL; 
	env->tempRatio   = env->temperature/env->T0;

	env->pressureRatio = pow( env->tempRatio, -env->gravity0*env->rho0*env->T0/( env->lapseRate*env->p0*144 ) );
    env->pressure      = env->p0*env->pressureRatio;

#if 0
	/* old way in case we need it for something */
	env->densityRatio = exp( -mo->altitudeMSL/36000 );
#else
	env->densityRatio = pow( env->tempRatio, -( env->gravity0*env->rho0*env->T0/( env->lapseRate*env->p0*144 ) + 1 ) );
	env->rho = env->rho0*env->densityRatio;
#endif

	/* gravity model */

	env->gravity = env->gravity0;

}

