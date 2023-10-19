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
#include <stdio.h>
#include "rmax/motion_ref.h"
#include "esim/util.h"
#include "rmax/motion_ref.h"
#include "rmax/gear_ref.h"
#include "rmax/gear.h"

#include "esim/quat.h"
#include "rmax/vector.h"
#include "rmax/matrix.h"
#include "rmax/navigation_ref.h"
#include "rmax/gcs_ref.h"

void gear( double terrainAlt,
		   struct vehicleMotion_ref *mo, 
	       struct state_ref *s, 
	       struct state_ref *sdot, double dt ) {

	struct params_gear_ref  *pg = mo->gear;
	struct motionXforms_ref *x  = mo->xforms;
	
	int i;
	
	double norm;
	
	/* for gear model */
	double *tx, *ty, *txdot, *tydot, *fz, *fzdot;
	double xu, yu, zu, g, fx, fy, uw, vw, ww, up, vp, wp, skid, xwd, ywd, zwd;
	double creep;
	
	/*************************************************************************/
	/* gear model */

	/* this model is not right for airplane axes! */
	
	pg->gearx = pg->geary = pg->gearz = 0.0;
	pg->gearl = pg->gearm = pg->gearn = 0.0;
	
	txdot = &sdot->tx0;
	tx    = &s->tx0;
	tydot = &sdot->ty0;
	ty    = &s->ty0;
	fzdot = &sdot->fz0;
	fz    = &s->fz0;
	
	for( i=0; i<NGEAR; i++ ) {
		*(txdot+i) = -10.0*(*(tx+i));
		*(tydot+i) = -10.0*(*(ty+i));
	}
	for( i=0; i<NCONTACTPOINTS; i++ ) {
		*(fzdot+i) = -10.0*(*(fz+i));
		pg->deflection[i] = 0;
	}

	if( pg->hmax > -s->p_b_e_L[2] - terrainAlt ) {   /* do ground interactions */

		/* find contact point frame relative to local (heading aligned with unsteered wheel) */
		norm = sqrt( SQ( x->dcm_bl[0][1] ) + SQ( x->dcm_bl[1][1] ) );
		if( norm>0.01 ) {
			pg->dcm_lp[1][0] = x->dcm_bl[0][1]/norm;
			pg->dcm_lp[1][1] = x->dcm_bl[1][1]/norm;
		} else {
			pg->dcm_lp[1][0] = 0;
			pg->dcm_lp[1][1] = 1;
		}
		pg->dcm_lp[1][2] = 0;
		pg->dcm_lp[0][0] = 0;
		pg->dcm_lp[0][1] = 0;
		pg->dcm_lp[0][2] = 1;
		v3cross( pg->dcm_lp[1], pg->dcm_lp[2], pg->dcm_lp[0] );

		/* transformation from body to contact point frame */
		mat_mult( (double *)pg->dcm_lp, 3, 3,
				  (double *) x->dcm_bl, 3, 3,
				  (double *)pg->dcm_bp );
		
		for( i=0; i<NCONTACTPOINTS; i++ ) {
			
			g = s->p_b_e_L[2] + terrainAlt + pg->xw[i]*x->dcm_bl[2][0] + pg->yw[i]*x->dcm_bl[2][1] + pg->zw[i]*x->dcm_bl[2][2];
			
			if( g > 0.0 ) {   /* point is hitting the ground */

				pg->deflection[i] = g;

				xwd = pg->xw[i] - g*x->dcm_bl[2][0]; /* point position deflected by the ground */
				ywd = pg->yw[i] - g*x->dcm_bl[2][1];
				zwd = pg->zw[i] - g*x->dcm_bl[2][2];
				
				uw = x->v_b_e_B[0] - ywd*s->w_b_e_B[2] + zwd*s->w_b_e_B[1]; /* velocity of the point wrt ground in body frame */
				vw = x->v_b_e_B[1] - zwd*s->w_b_e_B[0] + xwd*s->w_b_e_B[2];
				ww = x->v_b_e_B[2] - xwd*s->w_b_e_B[1] + ywd*s->w_b_e_B[0];
				
				up =  uw*pg->dcm_bp[0][0] + vw*pg->dcm_bp[0][1] + ww*pg->dcm_bp[0][2]; /* velocity of point in "wheel" coordinates */
				vp =  uw*pg->dcm_bp[1][0] + vw*pg->dcm_bp[1][1] + ww*pg->dcm_bp[1][2];
				wp =  uw*pg->dcm_bp[2][0] + vw*pg->dcm_bp[2][1] + ww*pg->dcm_bp[2][2];

				/* vertical reaction force in wheel frame */
				*(fzdot+i) = ( MAX( pg->kg[i]*( g + pg->Kdamp*wp ), 0.0 ) - *(fz+i) )/pg->tc;
				
				if( i<NGEAR ) {   /* this is a landing gear point */

					if( 0==i && pg->wheels ) { /* nose wheel steering */
						double upg, vpg;
						upg = up;  vpg = vp;
						up = +cos( mo->controls->noseWheelSteering )*upg + sin( mo->controls->noseWheelSteering )*vpg;
						vp = -sin( mo->controls->noseWheelSteering )*upg + cos( mo->controls->noseWheelSteering )*vpg;
					}

					if( pg->wheels ) { 
						if( ABS( up ) > dt*10.0 ) {
							fx = -pg->rollingFriction*(*(fz+i))*up/ABS( up );
						} else {
							if( dt > 0 ) fx = -pg->rollingFriction*(*(fz+i))*up/dt*0.1;
							else         fx = 0;
						}
						creep = MIN( ABS( up/pg->lpatch ), 10.0 );
						*(tydot+i) = -vp - creep*(*(ty+i));
						fy = pg->Ky*( *(ty+i) + 0.075*(*(tydot+i)) );
					} else {
						*(txdot+i) = -up;
						fx = pg->Kx*( *(tx+i) + 0.075*(*(txdot+i)) );
						*(tydot+i) = -vp;
						fy = pg->Ky*( *(ty+i) + 0.075*(*(tydot+i)) );
					}
					
					skid = pg->coeff*(*(fz+i))/sqrt( fx*fx + fy*fy + 0.001 );
					if( skid < 1.0 ) {
						fx *= skid;
						fy *= skid;
						*(txdot+i) *= skid;
						*(tydot+i) *= skid;
					}

					if( 0==i && pg->wheels ) { /* nose wheel steering */
						double fxg, fyg;
						fxg = fx;  fyg = fy;
						fx = +cos( mo->controls->noseWheelSteering )*fxg - sin( mo->controls->noseWheelSteering )*fyg;
						fy = +sin( mo->controls->noseWheelSteering )*fxg + cos( mo->controls->noseWheelSteering )*fyg;
					}

					
				} else {   /* this is not a landing gear point, simpler contact model */
					
					if( ABS( up ) > dt*10.0 ) fx = -pg->frcoeff*(*(fz+i))*up/ABS( up );
					else if( dt > 0 )         fx = -pg->frcoeff*(*(fz+i))*up/dt*0.1;
					else                      fx = 0;
					
					if( ABS( vp ) > dt*10.0 ) fy = -pg->frcoeff*(*(fz+i))*vp/ABS( vp );
					else if( dt > 0 )         fy = -pg->frcoeff*(*(fz+i))*vp/dt*0.1;
					else                      fy = 0;
					
				}

				xu =  fx*pg->dcm_bp[0][0] + fy*pg->dcm_bp[1][0] - (*(fz+i))*pg->dcm_bp[2][0]; /* force in body */
				yu =  fx*pg->dcm_bp[0][1] + fy*pg->dcm_bp[1][1] - (*(fz+i))*pg->dcm_bp[2][1];
				zu =  fx*pg->dcm_bp[0][2] + fy*pg->dcm_bp[1][2] - (*(fz+i))*pg->dcm_bp[2][2];

				pg->gearx += xu;
				pg->geary += yu;
				pg->gearz += zu;
				pg->gearl += zu*ywd - yu*zwd;
				pg->gearm += xu*zwd - zu*xwd;     
				pg->gearn += yu*xwd - xu*ywd;	
			}
		}
	}
}


/*
old code for collecting gear contact..no loger use
static void collectDeckContactStatsSetup(struct vehicleMotion_ref *mo,struct waterTarget2_ref *tgt){

	struct deckContactStat_ref*  stats = mo->gear->stats; 
	struct contactTrial_ref*     trial;
	int itrial,igear,i,istep;

	if (stats->init){
		stats->init   = 0;
		stats->itrial = 0;
		
		//clear everything
		for (itrial=0;itrial<MAX_CONTACT_TRIALS;itrial++){
			trial = stats->trial[itrial];

			for(igear=0;igear< NGEAR;igear++){

				trial->icontact[igear]    = 0;
				trial->lastCollect[igear] = 0;

				trial->pos[igear][0] = 0;
				trial->pos[igear][1] = 0;
				for(i=0;i<3;i++){
					trial->nf[igear][i]  = 0;
					for (istep=0;istep< MAX_COLLECT_STEPS;istep++){
						trial->nfCollect[igear][i][istep] = 0;
						trial->vel[igear][i][istep] = 0;
					}
				}

				for (istep=0;istep< MAX_COLLECT_STEPS;istep++){
					trial->time[igear][istep] = 0;
				}
			}
			
			for (istep=0;istep< MAX_COLLECT_STEPS;istep++){
				trial->mrz[istep] = 0;
			}
		}	
	}

	//hack to automatically turn the collection on after the first takeoff
	if (stats->enable==1 && stats->collect ==0 && (navout.wow ==0 || gcs1Outputs.wow ==0)){ //in case ob1 runs on a different instant
		stats->collect = 1;
	}

	//save the result into M-file when triggered
	if (stats->save){
		FILE* fp;

		stats->save = 0;

		fp = fopen("landingiLQR/gearStatsData.m","w");

		if (fp == NULL){
			printf("cannot open file\n");
		}else{
			
			//record position
			for (itrial=0;itrial<=MIN(stats->itrial,MAX_CONTACT_TRIALS-1);itrial++){
				trial = stats->trial[itrial];
				fprintf(fp,"gearPos(:,:,%d)=[\n",itrial+1);
				for(igear=0;igear< NGEAR;igear++){
					fprintf(fp,"%f\n%f\n",trial->pos[igear][0],trial->pos[igear][1]);			
				}
				fprintf(fp,"];\n");			
			}
			
			//record time
			for (itrial=0;itrial<=MIN(stats->itrial,MAX_CONTACT_TRIALS-1);itrial++){
				trial = stats->trial[itrial];
				fprintf(fp,"gearTime(:,:,%d)=[\n",itrial+1);
				for(igear=0;igear< NGEAR;igear++){
					for (istep=0;istep< MAX_COLLECT_STEPS;istep++){
						fprintf(fp,"%f ",trial->time[igear][istep]);	
					}
					fprintf(fp,"\n");
				}
				fprintf(fp,"];\n");		
			}

			//record main rotor force
			for (itrial=0;itrial<=MIN(stats->itrial,MAX_CONTACT_TRIALS-1);itrial++){
				trial = stats->trial[itrial];
				fprintf(fp,"mainRotorForce(:,%d)=[\n",itrial+1);
				for (istep=0;istep< MAX_COLLECT_STEPS;istep++){
					fprintf(fp,"%f\n",trial->mrz[istep]);	
				}
				fprintf(fp,"];\n");		
			}

			//record gear force
			for (itrial=0;itrial<=MIN(stats->itrial,MAX_CONTACT_TRIALS-1);itrial++){
				trial = stats->trial[itrial];
				fprintf(fp,"gearForce(:,:,%d)=[\n",itrial+1);
				for(igear=0;igear< NGEAR;igear++){
					for (i=0;i<3;i++){
						for (istep=0;istep< MAX_COLLECT_STEPS;istep++){
							fprintf(fp,"%f ",trial->nfCollect[igear][i][istep]);	
						}
						fprintf(fp,"\n");		
					}
				}
				fprintf(fp,"];\n");		
			}

			//record velocity
			for (itrial=0;itrial<=MIN(stats->itrial,MAX_CONTACT_TRIALS-1);itrial++){
				trial = stats->trial[itrial];
				fprintf(fp,"gearVel(:,:,%d)=[\n",itrial+1);
				for(igear=0;igear< NGEAR;igear++){
					for (i=0;i<3;i++){
						for (istep=0;istep< MAX_COLLECT_STEPS;istep++){
							fprintf(fp,"%f ",trial->vel[igear][i][istep]);	
						}
						fprintf(fp,"\n");		
					}
				}
				fprintf(fp,"];\n");		
			}

			fclose(fp);
		}
	}

	//move on to the next trial if.. 
	//collection is completed for all gears and all gears are no longer in contact with the deck...
	// All of this has to happen for some period of time
	//This is a quick and dirty code that works... you are welcome to clean it up
	if (stats->collect ==1 && stats->itrial < MAX_CONTACT_TRIALS){

		int readyToClear,igear;
		static int clearCnt =0;

	 	readyToClear = 1;
		for (igear=0;igear < NGEAR; igear++){
			readyToClear = readyToClear && (tgt->gearOnDeck[igear] == 0) && (stats->trial[stats->itrial]->icontact[igear] >=  MAX_COLLECT_STEPS);
		}

		if (readyToClear){
			clearCnt++;
		}

		if (clearCnt>100){
			clearCnt = 0;
			stats->itrial++;
		}
	}
}


static void collectDeckContactStats(struct vehicleMotion_ref *mo, int igear, double p_c_t_T[3], double f_B[3],double v_g_c_B[3]){

	int i;
	struct deckContactStat_ref*  stats = mo->gear->stats; 
	struct contactTrial_ref*     trial;
	
	if (stats->itrial >= MAX_CONTACT_TRIALS){ //kick out if too many are already collected
		return;
	}

	trial = stats->trial[stats->itrial];

	for (i=0;i<3;i++){//for live view of the force
		trial->nf[igear][i] = f_B[i]/(mo->mass->mass*mo->env->gravity);
	}
	
	if ((state.time - trial->lastCollect[igear] >= stats->collectDt) && //enough time has passed
		trial->icontact[igear] < MAX_COLLECT_STEPS){ //haven't fill up the storage

		//save the first position
		if (trial->icontact[igear] ==0){
			
			trial->pos[igear][0] = p_c_t_T[0]; 
			trial->pos[igear][1] = p_c_t_T[1];
		}	

		//save the normalized force
		for (i=0;i<3;i++){
			trial->nfCollect[igear][i][trial->icontact[igear]] = trial->nf[igear][i];
		}

		//save the velocity
		for (i=0;i<3;i++){
			trial->vel[igear][i][trial->icontact[igear]] = v_g_c_B[i];
		}

		//save the main rotor
		if (igear==0){
			trial->mrz[trial->icontact[igear]] = params_mr.mrz/(mo->mass->mass*mo->env->gravity);
		}

		//save the time
		trial->time[igear][trial->icontact[igear]] = state.time;
		trial->lastCollect[igear] = state.time;
		trial->icontact[igear]++;
	}
}
*/