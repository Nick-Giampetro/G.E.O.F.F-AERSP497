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
#include <math.h>

#include "esim/util.h"
#include "rmax/motion_ref.h"
#include "rmax/aeroMultirotor.h"

/******************************************************************************
 */

void aerodynamicsMultirotor( struct aeroMultirotor_ref *a,
                             struct vehicleMotion_ref *mo ) {

    struct env_ref            *env       = mo->env;
    struct motionXforms_ref   *x         = mo->xforms;

	/* fuse */
	a->xf_fus = 0.5*env->rho*a->xuu_fus*ABS( x->v_b_a_B[0] )*x->v_b_a_B[0];
	a->yf_fus = 0.5*env->rho*a->yvv_fus*ABS( x->v_b_a_B[1] )*x->v_b_a_B[1];
	a->zf_fus = 0.5*env->rho*a->zww_fus*ABS( x->v_b_a_B[2] )*x->v_b_a_B[2];
  
	/* sum */
	a->aerox = a->xf_fus;
	a->aeroy = a->yf_fus;
	a->aeroz = a->zf_fus;

}

/******************************************************************************
 */
