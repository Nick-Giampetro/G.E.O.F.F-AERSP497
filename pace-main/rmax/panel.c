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
/* Copyright (c) Eric N. Johnson, 1998.  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include "esim/db.h"
#include "esim/command.h"
#include "esim/util.h"
#include "esim/esim.h"
#include "esim/cnsl.h"
#include "esim/sim_ref.h"
#include "rmax/onboard_ref.h"
#include "rmax/gcs.h"
#include "rmax/joyinputs_ref.h"
#include "rmax/panel.h"
#include "rmax/motion_ref.h"

#define MODE_WINDOW 0
#define MODE_SUB 1

static struct panel_ref *whichPanel( void ) {

	/* get pointer to structure of window correspoding to handle */

	int win = glutGetWindow();
	if( win == gcs0Panel.win ) return &gcs0Panel;
	return &gcs0Panel;

}


/* stone this from plwin.c - really should make that part of the library at some point */
static float panelReadDataValue( Var *vp ) {

	switch( vp->type ) {
	case TYPE_DOUBLE:
		return (float)(*((double *)(vp->data)));
	case TYPE_FLOAT:
		return (float)(*((float *)(vp->data)));
	case TYPE_INT:
		return (float)(*((int *)(vp->data)));
	case TYPE_LONG:
		return (float)(*((long *)(vp->data)));
	case TYPE_ULONG:
		return (float)(*((unsigned long *)(vp->data)));
	case TYPE_CHAR:
		return (float)(*((char *)(vp->data)));
	case TYPE_UCHAR:
		return (float)(*((unsigned char *)(vp->data)));
	case TYPE_SCHAR:
		return (float)(*((signed char *)(vp->data)));
	case TYPE_SHORT:
		return (float)(*((short *)(vp->data)));
	case TYPE_USHORT:
		return (float)(*((unsigned short *)(vp->data)));
	case TYPE_UINT:
		return (float)(*((unsigned int *)(vp->data)));
	case TYPE_FLAG:
		return (float)(*((unsigned char *)(vp->data)));

	default:
		break;
	}

	return 0.0;

}


static void panelCheckINOP( struct panel_ref *p ) {

    /* need to redraw everything if link goes up/down */
	if( p->box[BOX_DATALINK]->state == 0 ||
		( sim.mode != SIM_MODE_RUN && onboard.run == 0 && p->redXwhenPaused ) /* this causes red x's to come up
														                         when GCS is paused when not a SITL sim */
		) {
		if( !p->inop ) {
			p->inop   = 1;
			p->redraw = 2;
		}
	} else {
		if( p->inop ) {
			p->inop   = 0;
			p->redraw = 2;
		}
	}
}


void readPanelStates( struct gcs_ref *g, struct gcsInstance_ref *gi ) {

	struct panel_ref          *p    = gi->panel;
    struct gcsSet_ref         *set  = gi->set;
	struct gcsDatalink_ref    *data = gi->datalink;
	struct motionControls_ref *co   = &motionControls;
	struct limitCheck_ref     *lc   = p->check;
	struct panelBox_ref       *b;

	int i;

    /* handle datalink status */

	if( sim.mode == SIM_MODE_INIT ) {
		p->lastUpdate1 = -1.0;
		p->lastUpdate2 = -1.0;
	}

	if( data->datalinkStatus1 ) {
        p->datalink1state = 1;
        p->lastUpdate1 = sim.time;
		data->datalinkStatus1 = 0;
    } else if( sim.time - p->lastUpdate1 > p->timeOut1 ) {
		p->datalink1state = 0;  /* timeout is over */
    }

	if( data->datalinkStatus2 ) {
        p->datalink2state = 1;
        p->lastUpdate2 = sim.time;
		data->datalinkStatus2 = 0;
    } else if( sim.time - p->lastUpdate2 > p->timeOut2 ) {
		p->datalink2state = 0;  /* timeout is over */
	}

	if(       p->datalink1state &&  data->m1->uplinkStatus[0] &&  p->datalink2state &&  data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 3;
	else if(  p->datalink1state &&  data->m1->uplinkStatus[0] && !p->datalink2state && !data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 1;
	else if( !p->datalink1state && !data->m1->uplinkStatus[0] &&  p->datalink2state &&  data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 2;
	else if(  p->datalink1state &&  data->m1->uplinkStatus[0] &&  p->datalink2state && !data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 12;
	else if(  p->datalink1state &&  data->m1->uplinkStatus[0] && !p->datalink2state &&  data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 11;
	else if(  p->datalink1state && !data->m1->uplinkStatus[0] &&  p->datalink2state &&  data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 10;
	else if(  p->datalink1state && !data->m1->uplinkStatus[0] &&  p->datalink2state && !data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 9;
	else if(  p->datalink1state && !data->m1->uplinkStatus[0] && !p->datalink2state &&  data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 8;
	else if(  p->datalink1state && !data->m1->uplinkStatus[0] && !p->datalink2state && !data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 7;
	else if( !p->datalink1state &&  data->m1->uplinkStatus[0] &&  p->datalink2state &&  data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 6;
	else if( !p->datalink1state &&  data->m1->uplinkStatus[0] &&  p->datalink2state && !data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 5;
	else if( !p->datalink1state && !data->m1->uplinkStatus[0] &&  p->datalink2state && !data->m1->uplinkStatus[1] )
		p->box[BOX_DATALINK]->state = 4;
	else
		p->box[BOX_DATALINK]->state = 0;

	panelCheckINOP( p );

	p->box[BOX_NAV]->state = LIMIT( data->navStatus, 0, 7 );

	p->box[BOX_PSPNAV]->state = LIMIT( data->m0->navStatus, 0, 2);

	p->box[BOX_GPS]->state = LIMIT( data->gpsStatus, 0, 7 );

	p->box[BOX_AGL]->state = LIMIT( data->aglStatus & 0x3, 0, 3 ); // For now, only concerned with status, not with sensor source

	if ( data->m0->LaunchState ) p->box[BOX_WOW]->state = 4;
	else  p->box[BOX_WOW]->state = 8;

	//p->box[BOX_WOW]->state = LIMIT( data->m0->wow, 0, 1 );
	//if(gi->outputs->fwingLaunchState != FWLAUNCHSTATE_NONE) {
	//	// In a fixed-wing launch state - we need to still be able to show armed, but this can override other modes
	//	if(gi->cntrlInput->arm->output == 1) {
	//		sprintf( p->box[BOX_WOW]->status6, "TO ARM" ); // Takeoff state/armed
	//		p->box[BOX_WOW]->state = 6;
	//	} else {
	//		// Show the launch state
	//		switch( gi->outputs->fwingLaunchState)
	//		{
	//		case FWLAUNCHSTATE_DELAY:
	//			sprintf( p->box[BOX_WOW]->status6, "TO DLY" );
	//			break;
	//		case FWLAUNCHSTATE_SPOOLUP:
	//			if( data->m0->motor == 1 ) {
	//				sprintf( p->box[BOX_WOW]->status6, "TO GO" );
	//			} else {
	//				sprintf( p->box[BOX_WOW]->status6, "TO %d", data->m0->motor - 1 );
	//			}
	//			break;
	//		case FWLAUNCHSTATE_LAUNCH:
	//			sprintf( p->box[BOX_WOW]->status6, "TO GO" );
	//			break;
	//		}
	//		p->box[BOX_WOW]->state = 6;
	//	}
	//} 
	//else if( p->box[BOX_WOW]->state == 0 && ( (gi->cntrlInput->arm->output == 1) /*ARM*/ || (data->m0->motor == 0) ) /*&& data->enable0*/ ) {
 //       p->box[BOX_WOW]->state = 2;
 //   } 
	//else if( ((p->box[BOX_WOW]->state == 1) || (p->box[BOX_WOW]->state == 3)) && (data->m0->motor > 0) ) {
 //       if( data->m0->motor == 1 ) {
 //           sprintf( p->box[BOX_WOW]->status4, "Gnd GO" );
 //       } else {
 //           sprintf( p->box[BOX_WOW]->status4, "Gnd %d", data->m0->motor - 1 );
 //       }
 //       p->box[BOX_WOW]->state = 4;
	//} 
	//else if((p->box[BOX_WOW]->state == 1) && (gi->cntrlInput->arm->output == 1)) {
	//	p->box[BOX_WOW]->state = 3;
	//}
	///* this one tells user if safed */
	//if( (p->box[BOX_WOW]->state == 1) && gi->traj->lockMotorStart ) p->box[BOX_WOW]->state = 5;
	///* on ground */
	//if( ( p->box[BOX_WOW]->state == 1 || p->box[BOX_WOW]->state == 3 || p->box[BOX_WOW]->state == 4 ) && (data->m0->wow == 2) ) p->box[BOX_WOW]->state = 7;
	///* unarmed, with or without SAS */
	//if( (data->m1->actuatorInterfaceStatus == 1) || (data->m1->actuatorInterfaceStatus == 3) ) {
	//	if( data->m0->wow ) p->box[BOX_WOW]->state = 8;
	//	else                p->box[BOX_WOW]->state = 9; /* unarmed in air mode!  this is bad... */
	//} 

	p->box[BOX_MAGNET]->state = data->magnetStatus;
	if( data->magnetStatus == 0 && data->m0->wow ) p->box[BOX_MAGNET]->state = 3; 

	p->box[BOX_FRAME]->state = LIMIT( data->overrun, 0, 1 );
	//data->overrun = 0;
	data->overrun = MAX(data->overrun-1, 0);

	p->box[BOX_DATARECORD]->state = data->m1->datarecordStatus;

	p->box[BOX_TRAJECTORY]->state = data->m1->safemode;

	p->box[BOX_BATTERY]->state = data->m1->batteryStatus;

    p->box[BOX_TX]->state = data->m1->tx;

    p->box[BOX_FUEL]->state = data->m1->fuel;

	p->box[BOX_YCS]->state = data->m1->ycsStatus;

    /*p->box[BOX_YAS]->state = data->m1->yasStatus;
    data->m1->yasStatus = 0;*/

    p->box[BOX_YRD]->state = data->m1->yrdStatus;

    p->box[BOX_RPM]->state = data->m1->hubStatus;
	if( gi->datalink->m0->wow ) p->box[BOX_RPM]->color[4] = BOX_GREEN;
	else                        p->box[BOX_RPM]->color[4] = BOX_RED;
    data->m1->hubStatus = 0;

    p->box[BOX_AUTO]->state = data->m0->autopilot;
		if ( 1 == p->box[BOX_AUTO]->state ) p->box[BOX_AUTO]->state = 2;

	p->box[BOX_UPLOAD]->state = gi->traj->uploadEach[gi->traj->edit];
	if( 1 == data->uploadingPlan ) p->box[BOX_UPLOAD]->state = 3;
	if( data->m1->lostComm )       p->box[BOX_UPLOAD]->state = 2;

    /*p->box[BOX_VISION]->state = data->trackingResults->mode;
    data->trackingResults->mode = 0;*/

    p->box[BOX_VISION]->state = data->m1->visionStatus;

	/* limit check */
	if( sim.mode == SIM_MODE_INIT || lc->init ) {
		lc->varp[0] = findVar( lc->var0 );
		lc->varp[1] = findVar( lc->var1 );
		lc->init = 0;
	}
	p->box[BOX_LIMITCHECK]->state = 0;
	for( i=0; i<LIMITCHECK_MAXCHECKS; i++ ) {
		if( lc->varp[i] != NULL ) {
			lc->value[i] = panelReadDataValue( lc->varp[i] );
			if( lc->value[i] < lc->lowerLimit[i] ||
				lc->value[i] > lc->upperLimit[i] ) {
				p->box[BOX_LIMITCHECK]->state += 1<<i;
			}
		}
	}

	p->box[BOX_OTHER]->state = LIMIT( data->m1->otherStatus, 0, MAXSTATUS );
	/*p->box[BOX_VISIONFORMATION]->state = data->m1->visionFormationStatus;
    data->m1->visionFormationStatus = 0;*/
	/*p->box[BOX_VISIONNAV]->state = data->m1->visionNavStatus;
    data->m1->visionNavStatus = 0;*/

	// For now, only report the status of the second AGL sensor.  Will incorporate more info later
	p->box[BOX_RANGEFINDER]->state = (data->m1->rangeFinderStatus >> 2)  & 0x3;
    data->m1->rangeFinderStatus = 0;
    p->box[BOX_ARDUPILOT]->state = data->m1->otherStatus;
    if( p->box[BOX_ARDUPILOT]->on ) data->m1->otherStatus = 0;

	/* flaps */
	if( data->m1->delc[0] == 0 ) {
		p->box[BOX_FLAP]->state = 0;
	} else {
		p->box[BOX_FLAP]->state = 1;
		sprintf( p->box[BOX_FLAP]->status1, "%d", (int)(data->m1->delc[0]*100) );
	}

	/* GPS reference */
	p->box[BOX_GPSREF]->state = data->gpsrefStatus;
	data->gpsrefStatus = 0;

	// handle history status
	p->box[BOX_HISTORY]->state = data->m1->historyStatus;
    data->m1->historyStatus = 0;
    
    switch( data->m1->missionStatus ){
        case 0: p->box[BOX_MISSION]->state=0; break;
        
        case 1:
        case 2: p->box[BOX_MISSION]->state=1; break;
        
        case 3:
        case 4: p->box[BOX_MISSION]->state=2; break;
        
        case 5:
        case 6: p->box[BOX_MISSION]->state=3; break;
        
        case 7:
        case 8: p->box[BOX_MISSION]->state=4; break;
        
        case 9:
        case 10: p->box[BOX_MISSION]->state=5; break;
        
        case 11: p->box[BOX_MISSION]->state=6; break;
        
        default:                        
			p->box[BOX_MISSION]->state = 15;
			switch( data->m1->missionStatus ) {
			default:
				sprintf( p->box[BOX_MISSION]->status15, "%d", data->m1->missionStatus );
				break;
			
			}
			break;
            
    }
      
    p->box[BOX_IMU]->state = data->m1->imuStatus;

	/* do timeout logic */
	for( i=0; i<NUMPANELBOXES; i++ ) {
		b = p->box[i];

		if( sim.mode == SIM_MODE_INIT ) {
			b->lastUpdate = -1.0;
		}

		if( b->timeOut >= 0.0 ) {
			if( b->state ) {
				b->lastUpdate = sim.time; /* remember last time state was true */
				b->rstate = b->state;
			} else {
				if( b->rstate ) { /* keep it on until timeout */
					if( sim.time - b->lastUpdate > b->timeOut ) {
						b->rstate = 0;  /* time out over */
					}
					b->state = b->rstate; 
				}
			}
		}
	}

	/* do sound effects */
	if( p->playSounds && sim.mode == SIM_MODE_RUN && sim.time > 1.0 ) {
		int playSound = 0;
		int anyRedYellow = 0;

		/* master caution */
		for( i=0; i<NUMPANELBOXES; i++ ) {
			b = p->box[i];
			if( b->on ) {
				if( b->color[b->state] == BOX_RED || b->color[b->state] == BOX_YELLOW ) {
					anyRedYellow = 1;
					if( b->color[b->state] != b->color[b->rstate_sound] ) playSound = 1;
				}
				b->rstate_sound = b->state;
			}
		}

#ifdef WIN32
		if( playSound )         PlaySound( "MasterCaution.wav", NULL, SND_ASYNC | SND_NOWAIT | SND_FILENAME );
		if( anyRedYellow == 0 ) PlaySound( NULL, NULL, SND_ASYNC | SND_NOWAIT );
#endif
	}

	/* determine overall status */

	p->status = 0;
	for( i=0; i<NUMPANELBOXES; i++ ) {
		b = p->box[i];
		if( b->on ) {
			if(        b->color[b->state] == BOX_RED    ) {
				p->status = MAX( p->status, 2 );
			} else if( b->color[b->state] == BOX_YELLOW ) {
				p->status = MAX( p->status, 1 );
			}
		}
	}

}


static void drawBitmapMessageCentered10( int x, int y, char *message ) {

	char *c = message;
	int width = 0;

	while( *c ) {
		width += glutBitmapWidth( GLUT_BITMAP_HELVETICA_10, *c );
		c++;
	}

	glRasterPos4f( (float)(x - width/2), (float)y, 0.0, 1.0 );
	while( *message ) {
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_10, *message );
		message++;
	}

}


static void drawBitmapMessageCentered18( int x, int y, char *message ) {

	char *c = message;
	int width = 0;

	while( *c ) {
		width += glutBitmapWidth( GLUT_BITMAP_HELVETICA_18, *c );
		c++;
	}

	glRasterPos4f( (float)(x - width/2), (float)y, 0.0, 1.0 );
	while( *message ) {
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, *message );
		message++;
	}

}


static void drawBitmapMessageCentered( int x, int y, char *message, char font ) {

	char *c = message;
	int width = 0;
	void *fp;

	switch( font ) {
		default:
		case 8:
			fp = GLUT_BITMAP_HELVETICA_18;
			break;
		case 7:
			fp = GLUT_BITMAP_HELVETICA_12;
			break;
		case 6:
			fp = GLUT_BITMAP_HELVETICA_10;
			break;
		case 5:
			fp = GLUT_BITMAP_TIMES_ROMAN_24;
			break;
		case 4:
			fp = GLUT_BITMAP_TIMES_ROMAN_10;
			break;
		case 3:
			fp = GLUT_BITMAP_8_BY_13;
			break;
		case 2:
			fp = GLUT_BITMAP_9_BY_15;
			break;
	}

	while( *c ) {
		width += glutBitmapWidth( fp, *c );
		c++;
	}

	glRasterPos4f( (float)(x - width/2), (float)y, 0.0, 1.0 );
	while( *message ) {
		glutBitmapCharacter( fp, *message );
		message++;
	}

}


static void drawBackBox( struct panel_ref *p, int mode ) {

	int x = 0;
	int y = 0;
	int w, h;

	w = p->boxw;
	h = p->boxh;

	if( w > 0 && h > 0 ) {

		if( mode == MODE_SUB ) {

			glBegin( GL_POLYGON );
			glVertex3f( (float) x,          (float)(y + 1), 0.0 );
			glVertex3f( (float) x,          (float)(y + h), 0.0 );
			glVertex3f( (float)(x + w - 1), (float)(y + h), 0.0 );
			glVertex3f( (float)(x + w - 1), (float)(y + 1), 0.0 );
			glEnd();

		} else { /* MODE_WINDOW */

			glBegin( GL_POLYGON );
			glVertex3f( (float)(x + 2),     (float)(y + 2),     0.0 );
			glVertex3f( (float)(x + 2),     (float)(y + h - 1), 0.0 );
			glVertex3f( (float)(x + w - 2), (float)(y + h - 1), 0.0 );
			glVertex3f( (float)(x + w - 2), (float)(y + 2),     0.0 );
			glEnd();

			glColor4fv( p->borderColor );
			glBegin( GL_LINE_STRIP );
			glVertex3f( (float)(x + 1),     (float)(y + h - 1), 0.0 );
			glVertex3f( (float)(x + 1),     (float)(y + 2),     0.0 );
			glVertex3f( (float)(x + w - 2), (float)(y + 2),     0.0 );
			glVertex3f( (float)(x + w - 2), (float)(y + h - 1), 0.0 );
			glVertex3f( (float)(x + 1),     (float)(y + h - 1), 0.0 );
			glEnd();

		}

	}

}


static void drawBox( struct panel_ref *p, struct panelBox_ref *b,
					 int boxnum, int mode, int winh ) {

	char *status;
	char buffer[25];
	char inop = 0;
	int shift;

	if( p->inop && b->inopFlag )
		inop = 1;

	b->state = LIMIT( b->state, 0, MAXSTATUS - 1 );

	glPushMatrix();

	if( mode == MODE_WINDOW )
		shift = 0;
	else
		shift = winh - ( p->nboxes - 1 )/p->perRow*p->boxh + p->shiftVert;

	glTranslatef( (boxnum%p->perRow)*p->boxw + p->shift,
		(float)((int)(boxnum/p->perRow)*p->boxh + shift),
		0 );

	if( inop ) {
		glColor4f( 0,0,0,1 );
	} else {
		if( b->color[b->state] == BOX_RED ) {
			if( fmod( sim.time, 2*p->blinkRate ) < p->blinkRate ) {
				glColor4fv( p->boxColor[BOX_RED] );
			} else {
				glColor4fv( p->boxColor[BOX_REDBLINK] );
			}
		} else {
			glColor4fv( p->boxColor[b->color[b->state]] );
		}
	}

	drawBackBox( p, mode );

	if( inop )
		glColor4f( .25f, .25f ,.25f, 1 );
	else
		glColor4fv( p->textColor[b->color[b->state]] );

	if( mode == MODE_WINDOW )
		drawBitmapMessageCentered10( p->boxw/2, p->sfontH, b->label );
	else
		strcpy( buffer, b->slabel );

	switch( b->state ) {
	default:
	case 0:  status = b->status0;  break;
	case 1:  status = b->status1;  break;
	case 2:  status = b->status2;  break;
	case 3:  status = b->status3;  break;
	case 4:  status = b->status4;  break;
	case 5:  status = b->status5;  break;
	case 6:  status = b->status6;  break;
	case 7:  status = b->status7;  break;
	case 8:  status = b->status8;  break;
	case 9:  status = b->status9;  break;
	case 10: status = b->status10; break;
	case 11: status = b->status11; break;
	case 12: status = b->status12; break;
	case 13: status = b->status13; break;
	case 14: status = b->status14; break;
	case 15: status = b->status15; break;
	}

	if( mode == MODE_WINDOW ) {
		drawBitmapMessageCentered18( p->boxw/2, p->bfontH, status );
	} else {
		if( strlen( buffer ) ) sprintf( buffer, "%s %s", buffer, status );
		else                   strcpy( buffer, status );
		drawBitmapMessageCentered( p->boxw/2, p->dfontH, buffer, p->sfont );
	}

	if( inop ) {
		glColor4fv( p->boxColor[BOX_RED] );
		glBegin( GL_LINES );
		glVertex3f(         3,            (float)(p->boxh - 2), 0.0 );
		glVertex3f( (float)(p->boxw - 3),         2,            0.0 );
		glVertex3f( (float)(p->boxw - 3), (float)(p->boxh - 2), 0.0 );
		glVertex3f(         3,                    2,            0.0 );
		glEnd();
	}

	glPopMatrix();

	if( mode == MODE_WINDOW ) b->olds = b->state;

}


static void justDraw( struct panel_ref *p, int mode, int winw, int winh ) {

	int i, nb;

	panelCheckINOP( p ); /* check again, in case paused */

	/* draw */

	if( mode == MODE_WINDOW ) {
		p->boxw = p->wboxw;
		p->boxh = p->wboxh;
	} else { /* MODE_SUB */
		p->boxw = p->sboxw;
		p->boxh = p->sboxh;
	}

	p->perRow = MAX( 1, winw/p->boxw );
	p->shift  = (float)(fmod( winw, p->boxw )*0.5);

	p->nboxes = 0;
	for( i=0; i<NUMPANELBOXES; i++ ) {
		if( p->box[i]->on ) p->nboxes++;
	}

	if( p->redraw && mode == MODE_WINDOW ) {

		p->redraw = MAX( 0, p->redraw - 1 );

		glClear( GL_COLOR_BUFFER_BIT );

		nb = 0;
		for( i=0; i<NUMPANELBOXES; i++ ) {
			if( p->box[i]->on ) drawBox( p, p->box[i], nb++, mode, winh );
		}

	} else {

		nb = 0;
		for( i=0; i<NUMPANELBOXES; i++ ) {
			if( ( p->box[i]->olds != p->box[i]->state || p->forceDraw || mode == MODE_SUB ) &&
				  p->box[i]->on ) {
				drawBox( p, p->box[i], nb++, mode, winh );
			}
		}

	}

}


int redrawPanel( void ) {

	struct panel_ref *p;
	p = whichPanel();

	justDraw( p, MODE_WINDOW, p->winw, p->winh );

	glutSwapBuffers();

	return 0;

}


void panelSubDraw( struct panel_ref *p, int winw, int winh ) {

	justDraw( p, MODE_SUB, winw, winh );

}


void redrawPanelCallback( void ) {

	redrawPanel();

}


void panelReshape( int w, int h ) {

	struct panel_ref *p;
	p = whichPanel();

	p->winw = w;
	p->winh = h;

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glOrtho( 0, p->winw, 0, p->winh, -1, 1 );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glTranslatef( 0, (float)p->winh, 0 );
	glRotatef( 180, 1, 0, 0 );

	glViewport( 0, 0, p->winw, p->winh );

	p->redraw = 2;

	redrawPanel();

}


void panelMouseButton( int button, int state, int x, int y ) {

	struct panel_ref *p;
	p = whichPanel();

	if( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN ) {
	}
	if( button == GLUT_LEFT_BUTTON && state == GLUT_UP ) {
		p->redraw = 2;
		glutPostWindowRedisplay( p->win );
	}

}


void panelMouseMotion( int x, int y ) {

	struct panel_ref *p;
	p = whichPanel();

	if( sim.mode == SIM_MODE_PAUSE )
		glutPostWindowRedisplay( p->win );

}


void panelMenuState( int status ) {

	if( status == GLUT_MENU_IN_USE )
		sim.skipTime = 1;

}


void panelKeyboard( unsigned char key, int x, int y ) {

	struct panel_ref *p;
	p = whichPanel();

	switch( key ) {
	case 's':
	case 'S':
		break;
	case '@':
		commandExecute( "@" );
		break;
	case 0x1b:
		p->open = 0;
		glutDestroyWindow( p->win );
		return;
		break;
	default:
		break;
	}

	glutPostWindowRedisplay( p->win );

}


void panelSpecialKeys( int key, int x, int y ) {

	struct panel_ref *p;
	p = whichPanel();

	switch( key ) {
	case GLUT_KEY_F1:
		break;
	case GLUT_KEY_LEFT:
		break;
	case GLUT_KEY_RIGHT:
		break;
	case GLUT_KEY_UP:
		break;
	case GLUT_KEY_DOWN:
		break;
	default:
		break;
	}

	glutPostWindowRedisplay( p->win );

}


void panelMainMenu( int value ) {

	struct panel_ref *p;
	p = whichPanel();

	switch( value ) {
	case 1:
		p->redraw = 2;
		glutPostWindowRedisplay( p->win );
		break;
	case 2:
		p->open = 0;
		glutDestroyWindow( p->win );
		return;
		break;
	default:
		break;
	}

	glutPostWindowRedisplay( p->win );

}


void panelVisibility( int visibility ) {

	struct panel_ref *p;
	p = whichPanel();

	p->visibility = visibility;

}


static void updateAPanel( struct panel_ref *p ) {

	if( p->open )
		if( p->visibility == GLUT_VISIBLE )
			glutPostWindowRedisplay( p->win );

}


void updatePanel( void ) {

	updateAPanel( &gcs0Panel );

}


void openPanel( struct panel_ref *p ) {

	if( !p->open ) {

		p->open = 1;
		p->check->init = 1;

		/* glut initialize */

		glutInitWindowSize( p->winw, p->winh );
		glutInitWindowPosition( p->x, p->y );
		glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
		p->win = glutCreateWindow( p->name );

		/* top level call-backs */

		glutDisplayFunc( redrawPanelCallback );
		glutReshapeFunc( panelReshape );
		glutMouseFunc( panelMouseButton );
		glutMotionFunc( panelMouseMotion );
		glutPassiveMotionFunc( panelMouseMotion );
		glutKeyboardFunc( panelKeyboard );
		glutSpecialFunc( panelSpecialKeys );
		glutVisibilityFunc( panelVisibility );

		/* menu setup */

		glutCreateMenu( panelMainMenu );
		glutAddMenuEntry( "Refresh", 1 );
		glutAddMenuEntry( "Close Window", 2 );

		glutMenuStateFunc( panelMenuState );
		glutAttachMenu( GLUT_RIGHT_BUTTON );

		glEnable( GL_CULL_FACE );
		glEnable( GL_NORMALIZE );
		glDisable( GL_LIGHTING );
		glClearColor( p->bkgdColor[0], p->bkgdColor[1],
			p->bkgdColor[2], p->bkgdColor[3] );
		glDisable( GL_BLEND );
		glLineWidth( 1.0 );
		/*if( p->forceDraw ) {
			glDrawBuffer( GL_BACK );
		} else {
			glDrawBuffer( GL_FRONT_AND_BACK );
		}*/

		panelReshape( p->winw, p->winh );

	}

}


static void closePanelCmd( int window ) {

	struct panel_ref *p = NULL;

	switch( window ) {
	case 0:  p = &gcs0Panel;  break;
	default:  break;
	}

	if( p != NULL ) {

		if( p->open ) {
			p->open = 0;
			glutDestroyWindow( p->win );
		}

	}

}


static void openPanelCmd( int window ) {

	struct panel_ref *p = NULL;

	switch( window ) {
	case 0:  p = &gcs0Panel;  break;
	default:  break;
	}

	if( p != NULL ) {

		if( p->open ) {
			p->open = 0;
			glutDestroyWindow( p->win );
		}

		openPanel( p );
	}

}


void commandPanel( int argc, char **argv ) {

	if(        argc == 3 && !strcmp( argv[2], "close" )  ) {
		closePanelCmd( atoi( argv[1] ) );
	} else if( argc == 2 && !strcmp( argv[1], "close" ) ) {
		closePanelCmd( gcs.active );
	} else if( argc == 2 ) {
		openPanelCmd( atoi( argv[1] ) );
	} else {
		openPanelCmd( gcs.active );
	}

}


static void initAPanel( struct panel_ref *p ) {

	struct panelBox_ref *b;

	/* set up boxes */

	/* datalink */

	b = p->box[BOX_DATALINK];
	sprintf( b->label,  "Datalink" );
	sprintf( b->slabel, "Com" );
	sprintf( b->status0,  "DOWN"  );  b->color[0]  = BOX_RED;
	sprintf( b->status1,  "LINK1" );  b->color[1]  = BOX_GREEN;
	sprintf( b->status2,  "LINK2" );  b->color[2]  = BOX_GREEN;
	sprintf( b->status3,  "BOTH"  );  b->color[3]  = BOX_DARKGREEN;
	sprintf( b->status4,  "2Rx"   );  b->color[4]  = BOX_YELLOW;
	sprintf( b->status5,  "1T+2R" );  b->color[5]  = BOX_GREEN;
	sprintf( b->status6,  "1Tx+2" );  b->color[6]  = BOX_GREEN;
	sprintf( b->status7,  "1Rx"   );  b->color[7]  = BOX_YELLOW;
	sprintf( b->status8,  "1R+2T" );  b->color[8]  = BOX_GREEN;
	sprintf( b->status9,  "1R+2R" );  b->color[9]  = BOX_YELLOW;
	sprintf( b->status10, "1Rx+2" );  b->color[10] = BOX_GREEN;
	sprintf( b->status11, "1+2Tx" );  b->color[11] = BOX_GREEN;
	sprintf( b->status12, "1+2Rx" );  b->color[12] = BOX_GREEN;
	b->timeOut = -1.0; /* datalink handles timeouts in a special way (becuase there are 2) */
	b->inopFlag = 0;

	/* nav */

	b = p->box[BOX_NAV];
	sprintf( b->label,  "Navigation" );
	sprintf( b->slabel, "Nav" );
	sprintf( b->status0, "INIT"  );  b->color[0] = BOX_ORANGE;
	sprintf( b->status1, "RUN"   );  b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "BIT"   );  b->color[2] = BOX_ORANGE;
	sprintf( b->status3, "NoGPS" );  b->color[3] = BOX_YELLOW;
    sprintf( b->status4, "GPSALT" );  b->color[4] = BOX_YELLOW;
    sprintf( b->status5, "Alt+VINS" );  b->color[5] = BOX_GREEN;
	sprintf( b->status6, "FAIL"  );  b->color[6] = BOX_RED;
    sprintf( b->status7, "VINS" );  b->color[7] = BOX_GREEN;
	b->timeOut = -1.0;

	/* gps */

	b = p->box[BOX_GPS];
	sprintf( b->label,  "GPS" );
	sprintf( b->slabel, "GPS" );
	sprintf( b->status0, "OFF"    );  b->color[0] = BOX_RED;
	sprintf( b->status1, "NoLock" );  b->color[1] = BOX_RED;
	sprintf( b->status2, "GOOD"   );  b->color[2] = BOX_GREEN;
	sprintf( b->status3, "DIFF"   );  b->color[3] = BOX_GREEN;
	sprintf( b->status4, "RTK"    );  b->color[4] = BOX_DARKGREEN;
	sprintf( b->status5, "TC"     );  b->color[5] = BOX_ORANGE;
	sprintf( b->status6, "Align+" );  b->color[6] = BOX_DARKGREEN;
	sprintf( b->status7, "Align"  );  b->color[7] = BOX_DARKGREEN;
	b->timeOut = -1.0;

	/* agl */

	b = p->box[BOX_AGL];
	sprintf( b->label,  "AGL" );
	sprintf( b->slabel, "AGL" );
	sprintf( b->status0, "OFF"   );  b->color[0] = BOX_RED;
	sprintf( b->status1, "RANGE" );  b->color[1] = BOX_GREEN;
	sprintf( b->status2, "GOOD"  );  b->color[2] = BOX_DARKGREEN;
	sprintf( b->status3, "OUT"   );  b->color[3] = BOX_GREEN; // Outlier
	b->timeOut = 2.5;

	/* ground/air mode, weight on wheels (WOW) */

	b = p->box[BOX_WOW];
	sprintf( b->label,  "WOW" );
	sprintf( b->slabel, "" );
	sprintf( b->status0, "AIR"     );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "Ground"  );  b->color[1] = BOX_GREEN;
	sprintf( b->status2, "ARM/Air" );  b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "ARM/Gnd" );  b->color[3] = BOX_GREEN;
	sprintf( b->status4, "ARM"  );     b->color[4] = BOX_GREEN;
	sprintf( b->status5, "SAFED"   );  b->color[5] = BOX_ORANGE;
	sprintf( b->status6, "TO DLY"  );  b->color[6] = BOX_GREEN;
	sprintf( b->status7, "Ground"  );  b->color[7] = BOX_YELLOW;
	sprintf( b->status8, "UNARMED" );  b->color[8] = BOX_ORANGE;
	sprintf( b->status9, "UNARM/A" );  b->color[9] = BOX_RED;
	b->timeOut = -1.0;

	/* magnet */

	b = p->box[BOX_MAGNET];
	sprintf( b->label,  "Compass" );
	sprintf( b->slabel, "Mag" );
	sprintf( b->status0, "OFF" );  b->color[0] = BOX_YELLOW;
	sprintf( b->status1, "ON"  );  b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "CAL" );  b->color[2] = BOX_ORANGE;
	sprintf( b->status3, "OFF" );  b->color[3] = BOX_RED;
	b->timeOut = -1.0;

	/* frame overruns */

	b = p->box[BOX_FRAME];
	sprintf( b->label,  "Frames" );
	sprintf( b->slabel, "CPU" );
	sprintf( b->status0, "GOOD" );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "SLOW" );  b->color[1] = BOX_GREEN;
	b->timeOut = -1.0;

	/* onboard data recording */

	b = p->box[BOX_DATARECORD];
	sprintf( b->label,  "Data Record" );
	sprintf( b->slabel, "DR" );
	sprintf( b->status0, "OFF"     );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "ON"      );  b->color[1] = BOX_GREEN;
	sprintf( b->status2, "FULL"    );  b->color[2] = BOX_ORANGE;
	sprintf( b->status3, "DWNLD"   );  b->color[3] = BOX_ORANGE;
	sprintf( b->status4, "FL FULL" );  b->color[4] = BOX_ORANGE;
    sprintf( b->status5, "CLEAR"   );  b->color[5] = BOX_GREEN;
		b->on = 0;

	/* onboard trajectory */

	b = p->box[BOX_TRAJECTORY];
	sprintf( b->label,  "Trajectory" );
	sprintf( b->slabel, "" );
	sprintf( b->status0,  "GO"      );  b->color[0]  = BOX_DARKGREEN;
	sprintf( b->status1,  "STOP"    );  b->color[1]  = BOX_GREEN;
	sprintf( b->status2,  "GO 2"    );  b->color[2]  = BOX_GREEN;
	sprintf( b->status3,  "SLOW"    );  b->color[3]  = BOX_GREEN;
	sprintf( b->status4,  "GCAS"    );  b->color[4]  = BOX_GREEN;
	sprintf( b->status5,  "(s)STOP" );  b->color[5]  = BOX_GREEN;
	sprintf( b->status6,  "GCAS"    );  b->color[6]  = BOX_YELLOW;
	sprintf( b->status7,  "NOE"     );  b->color[7]  = BOX_ORANGE;
	sprintf( b->status8,  "PANIC"   );  b->color[8]  = BOX_YELLOW;
	sprintf( b->status9,  "AVOID"   );  b->color[9]  = BOX_ORANGE;
	sprintf( b->status10, "FENCE"   );  b->color[10] = BOX_GREEN;
	sprintf( b->status11, "FENCE"   );  b->color[11] = BOX_YELLOW;
	sprintf( b->status12, "STATION" );  b->color[12] = BOX_GREEN;
	sprintf( b->status13, "FLAND"   );  b->color[13] = BOX_GREEN;
	sprintf( b->status14, "FLAND MPC"); b->color[14] = BOX_GREEN;
	b->on = 0;

	/* battery voltage */

	b = p->box[BOX_BATTERY];
	sprintf( b->label,  "Battery" );
	sprintf( b->slabel, "Batt" );
	sprintf( b->status0, "GOOD" );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "LOW"  );  b->color[1] = BOX_YELLOW;
	sprintf( b->status2, "HIGH" );  b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "DEAD" );  b->color[3] = BOX_RED;
	b->on = 0;

	/* yamaha receiving servo commands */

	b = p->box[BOX_TX];
	sprintf( b->label,  "Fail Safe" );
	sprintf( b->slabel, "Srv" );
	sprintf( b->status0, "GOOD" );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "FAIL" );  b->color[1] = BOX_RED;
	b->on = 0;

	/* fuel */

	b = p->box[BOX_FUEL];
	sprintf( b->label,  "Fuel" );
	sprintf( b->slabel, "Gas" );
	sprintf( b->status0, "GOOD"  );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "LOW"   );  b->color[1] = BOX_YELLOW;
	sprintf( b->status2, "ERROR" );  b->color[2] = BOX_YELLOW;
	b->on = 0;

	/* YCS */

	b = p->box[BOX_YCS];
	sprintf( b->label,  "YCS" );
	sprintf( b->slabel, "YCS" );
	sprintf( b->status0, "OFF"   );  b->color[0] = BOX_RED;
	sprintf( b->status1, "GOOD"  );  b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "START" );  b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "OFF"   );  b->color[3] = BOX_ORANGE;
	sprintf( b->status4,  "STOP"   );  b->color[4]  = BOX_GREEN;      /* SVI only */
	sprintf( b->status5,  "IDLE"   );  b->color[5]  = BOX_GREEN;      /* SVI only */
	sprintf( b->status6,  "FLIGHT" );  b->color[6]  = BOX_DARKGREEN;  /* SVI only */
	sprintf( b->status7,  "ABORT"  );  b->color[7]  = BOX_RED;        /* SVI only */
	sprintf( b->status8,  "TEST"   );  b->color[8]  = BOX_ORANGE;     /* SVI only */
	sprintf( b->status9,  "?"      );  b->color[9]  = BOX_ORANGE;     /* SVI only */
	sprintf( b->status10, ">STOP"  );  b->color[10] = BOX_GREEN;      /* SVI only */
	sprintf( b->status11, ">IDLE"  );  b->color[11] = BOX_GREEN;      /* SVI only */
	sprintf( b->status12, ">FLT"   );  b->color[12] = BOX_GREEN;      /* SVI only */
	sprintf( b->status13, ">ABORT" );  b->color[13] = BOX_RED;        /* SVI only */
	b->on = 0;

	/* YAS */

	/*b = p->box[BOX_YAS];
	sprintf( b->label,  "YAS" );
	sprintf( b->slabel, "YAS" );
	sprintf( b->status0, "OFF" );  b->color[0] = BOX_RED;
	sprintf( b->status1, "ON"  );  b->color[1] = BOX_DARKGREEN;
	b->timeOut = 2.0;*/

	/* YRD */

	b = p->box[BOX_YRD];
	sprintf( b->label,  "RC" );
	sprintf( b->slabel, "RC" );
	sprintf( b->status0, "OFF"   );  b->color[0] = BOX_RED;
	sprintf( b->status1, "GOOD"  );  b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "YACS"  );  b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "RANGE" );  b->color[3] = BOX_RED;
	sprintf( b->status4, "OFF"   );  b->color[4] = BOX_ORANGE;

	/* RPM */

	b = p->box[BOX_RPM];
	sprintf( b->label,  "RPM" );
	sprintf( b->slabel, "RPM" );
	sprintf( b->status0, "OFF"  );  b->color[0] = BOX_DARKGREEN; /* while not using RPM sensor */
	sprintf( b->status1, "HOLD" );  b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "OPEN" );  b->color[2] = BOX_GREEN;
	sprintf( b->status3, "OK" );    b->color[3] = BOX_DARKGREEN;
	sprintf( b->status4, "LOW" );   b->color[4] = BOX_RED;
	sprintf( b->status5, "HIGH" );  b->color[5] = BOX_RED;
	b->timeOut = 2.0;
	b->on = 0;

	/* Autopilot */

	b = p->box[BOX_AUTO];
	sprintf( b->label,  "Autopilot" );
	sprintf( b->slabel, "" );
	sprintf( b->status0, "MANUAL"    );  b->color[0]  = BOX_ORANGE;
	sprintf( b->status1, "MANUAL."   );  b->color[1]  = BOX_ORANGE;
	sprintf( b->status2, "AUTO"      );  b->color[2]  = BOX_DARKGREEN;
	sprintf( b->status3, "AUTO."     );  b->color[3]  = BOX_DARKGREEN;
	sprintf( b->status4, "CntrChk"   );  b->color[4]  = BOX_ORANGE;
	sprintf( b->status5, "CntrChk."  );  b->color[5]  = BOX_ORANGE;
	sprintf( b->status6, "AutoNoGPS" );  b->color[6]  = BOX_YELLOW;
	sprintf( b->status7, "AUTO_ONB"  );  b->color[7]  = BOX_DARKGREEN; // Intended for PSP : flying using onboard controller
	sprintf( b->status8, "AUTO_OFFB" );  b->color[8]  = BOX_GREEN;// Intended for PSP : flying using offboard controller
	sprintf( b->status9, "IDLE"      );  b->color[9]  = BOX_ORANGE; // Intended for PSP : vehicle with motor idling/off and set for communication only
	sprintf( b->status10, "AugSP"    );  b->color[10] = BOX_ORANGE; // Flying using safety pilot radio link, but the DSP still is providing manual augmentation mode
	sprintf( b->status11, "AugSP."   );  b->color[11] = BOX_ORANGE; // Flying using safety pilot radio link, but the DSP still is providing manual augmentation mode
	sprintf( b->status12, "AugSPNoG." ); b->color[12] = BOX_ORANGE; // Flying using safety pilot radio link, but the DSP still is providing manual augmentation mode
	sprintf( b->status13, "RatefSP"  );  b->color[13] = BOX_ORANGE; // Flying using safety pilot radio link, but the DSP still is providing manual rate feedback mode
	sprintf( b->status14, "RatefSP." );  b->color[14] = BOX_ORANGE; // Flying using safety pilot radio link, but the DSP still is providing manual rate feedback mode

	/* Upload */

	b = p->box[BOX_UPLOAD];
	sprintf( b->label,  "Plan" );
	sprintf( b->slabel, "Plan" );
	sprintf( b->status0, "GOOD"   );   b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "Upload" );   b->color[1] = BOX_GREEN;
	sprintf( b->status2, "Recover"  ); b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "Loading" );  b->color[3] = BOX_GREEN;
	b->on = 0;

	/* Vision on onboard2 */

	/*b = p->box[BOX_VISION];
	sprintf( b->label,  "Vision" );
	sprintf( b->slabel, "Vis" );
	sprintf( b->status0, "OFF"   );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "ON(B)" );  b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "ON(W)" );  b->color[2] = BOX_DARKGREEN;
	sprintf( b->status3, "ON(S)" );  b->color[3] = BOX_DARKGREEN;
	sprintf( b->status4, "ON(F)" );  b->color[4] = BOX_GREEN;
	b->timeOut = 10.0;*/

    b = p->box[BOX_VISION];
    sprintf( b->label,  "Vision" );
    sprintf( b->slabel, "Vis" );
    sprintf( b->status0, "OFF" );      b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "NoLock" );   b->color[1] = BOX_YELLOW;
    sprintf( b->status2, "LOCK" );     b->color[2] = BOX_GREEN;
    sprintf( b->status3, "Set TG" );   b->color[3] = BOX_ORANGE;
    sprintf( b->status4, "Set AR" );   b->color[4] = BOX_ORANGE;
    sprintf( b->status5, "DOWN" );     b->color[5] = BOX_YELLOW;
    b->timeOut = 2.0;
		b->on = 0;

	/* amgl */

	/*b = p->box[BOX_AMGL];
	sprintf( b->label,  "AMGL" );
	sprintf( b->slabel, "AMG" );
	sprintf( b->status0, "OFF" );    b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "OK" );     b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "AVOID" );  b->color[2] = BOX_RED;
    b->timeOut = 2.0;*/

	/* Lavoid or Limit check */

	b = p->box[BOX_LIMITCHECK];
	sprintf( b->label,  "Limit Check" );
	sprintf( b->slabel, "Lim" );
	sprintf( b->status0, "GOOD" );   b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "0 Out" );  b->color[1] = BOX_RED;
	sprintf( b->status2, "1 Out" );  b->color[2] = BOX_RED;
	sprintf( b->status3, "01 Out" ); b->color[3] = BOX_RED;
	b->on = 0;

	/* airdata sensor */

	/*b = p->box[BOX_AIRDATA];
	sprintf( b->label,  "AirData" );
	sprintf( b->slabel, "Air" );
	sprintf( b->status0, "OFF" );   b->color[0] = BOX_DARKGREEN;
	sprintf( b->status2, "GOOD" );  b->color[2] = BOX_DARKGREEN;
	sprintf( b->status3, "BAD" );   b->color[3] = BOX_RED;
	b->timeOut = 1.5;*/

	/*b = p->box[BOX_HURT];
	sprintf( b->label,  "HURT" );
	sprintf( b->slabel, "Hrt" );
	sprintf( b->status0, "OFF" );    b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "Tx" );     b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "Rx/Tx" );  b->color[2] = BOX_GREEN;
	sprintf( b->status3, "REQ" );    b->color[3] = BOX_GREEN;
	b->inopFlag = 0;*/

	b = p->box[BOX_TRANSPONDER];
	sprintf( b->label,  "ADS-B" );
	sprintf( b->slabel, "ADSB" );
	sprintf( b->status0, "OFF" );    b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "ON"  );    b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "SBY" );    b->color[2] = BOX_GREEN;
	sprintf( b->status3, "TX"  );    b->color[3] = BOX_DARKGREEN;
	sprintf( b->status4, "ALT" );    b->color[4] = BOX_DARKGREEN;
	b->on = 0;
	b->timeOut = 5.0;

	b = p->box[BOX_OTHER];
    sprintf( b->label,  "SVH" );
    sprintf( b->slabel, "SVH" );
    sprintf( b->status0, "OFF" );     b->color[0]  = BOX_DARKGREEN;
    sprintf( b->status1, "OK" );      b->color[1]  = BOX_DARKGREEN;
    sprintf( b->status2, "ERR" );     b->color[2]  = BOX_YELLOW;
    sprintf( b->status3, "3"   );     b->color[3]  = BOX_ORANGE;
    sprintf( b->status4, "4"   );     b->color[4]  = BOX_ORANGE;
    sprintf( b->status5, "5"   );     b->color[5]  = BOX_ORANGE;
    sprintf( b->status6, "6"   );     b->color[6]  = BOX_ORANGE;
    sprintf( b->status7, "7"   );     b->color[7]  = BOX_ORANGE;
    sprintf( b->status8, "8"   );     b->color[8]  = BOX_ORANGE;
    sprintf( b->status9, "9"   );     b->color[9]  = BOX_ORANGE;
    sprintf( b->status10, "10" );     b->color[10] = BOX_ORANGE;
    sprintf( b->status11, "11" );     b->color[11] = BOX_ORANGE;
    sprintf( b->status12, "12" );     b->color[12] = BOX_ORANGE;
    sprintf( b->status13, "13" );     b->color[13] = BOX_ORANGE;
    sprintf( b->status14, "14" );     b->color[14] = BOX_ORANGE;
    sprintf( b->status15, "15" );     b->color[15] = BOX_ORANGE;
		b->on = 0;

	/*b = p->box[BOX_VISIONNAV];
    sprintf( b->label,  "VisionNav" );
    sprintf( b->slabel, "VN" );
    sprintf( b->status0, "OFF" );     b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "NoLock" );  b->color[1] = BOX_YELLOW;
    sprintf( b->status2, "NoNav" );   b->color[2] = BOX_RED;
    sprintf( b->status3, "GOOD" );    b->color[3] = BOX_GREEN;
    b->timeOut = 2.0;*/

	b = p->box[BOX_HISTORY];
	sprintf( b->label,  "History" );
	sprintf( b->slabel, "Hist" );
	sprintf( b->status0, "OFF" );    b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "GCS" );    b->color[1] = BOX_GREEN;
	sprintf( b->status2, "1" );      b->color[2] = BOX_GREEN;
	sprintf( b->status3, "GCS/1" );  b->color[3] = BOX_GREEN;
	sprintf( b->status4, "2" );      b->color[4] = BOX_GREEN;
	sprintf( b->status5, "GCS/2" );  b->color[5] = BOX_GREEN;
	sprintf( b->status6, "1/2" );    b->color[6] = BOX_GREEN;
	sprintf( b->status7, "ALL" );    b->color[7] = BOX_GREEN;
    b->timeOut = 2.0;
		b->on = 0;

	/* mission manager */

	b = p->box[BOX_MISSION];
	sprintf( b->label,  "Mission" );
	sprintf( b->slabel, "MO" );
	sprintf( b->status0, "OFF" );      b->color[0]  = BOX_DARKGREEN;
	sprintf( b->status1, "Entry" );    b->color[1]  = BOX_GREEN;
	sprintf( b->status2, "Search" );   b->color[2]  = BOX_GREEN;
	sprintf( b->status3, "Loiter" );   b->color[3]  = BOX_GREEN;
	sprintf( b->status4, "Pickup" );   b->color[4]  = BOX_GREEN;
	sprintf( b->status5, "Leave" );    b->color[5]  = BOX_GREEN;
	sprintf( b->status6, "Dstr" );     b->color[6]  = BOX_GREEN;
    sprintf( b->status7, "Hvr" );      b->color[7]  = BOX_GREEN;
    sprintf( b->status8, "srchTgt" );  b->color[8]  = BOX_GREEN;
    sprintf( b->status9, "srchHeli" ); b->color[9]  = BOX_GREEN;
    sprintf( b->status10, "stby" );    b->color[10] = BOX_GREEN;
    sprintf( b->status11, "Land" );    b->color[11] = BOX_GREEN;
    sprintf( b->status15, "???" );     b->color[15] = BOX_GREEN;
		b->on = 0;

	/* mission manager */

	b = p->box[BOX_MISSIONGCS];
	sprintf( b->label,  "MissionG" );
	sprintf( b->slabel, "MG" );
	sprintf( b->status0, "OFF" );      b->color[0]  = BOX_DARKGREEN;
	sprintf( b->status5, "Init" );     b->color[5]  = BOX_GREEN;
	sprintf( b->status6, "Launch" );   b->color[6]  = BOX_GREEN;
	sprintf( b->status7, "Run" );      b->color[7]  = BOX_GREEN;
	b->inopFlag = 0;
	b->on = 0;

	/* mission manager */

	b = p->box[BOX_EPM];
	sprintf( b->label,  "EPM" );
	sprintf( b->slabel, "EPM" );
	sprintf( b->status0, "off off" );  b->color[0]  = BOX_GREEN;
	sprintf( b->status1, "Touch" );    b->color[1]  = BOX_GREEN;
	sprintf( b->status2, "Grab" );     b->color[2]  = BOX_ORANGE;
	sprintf( b->status3, "on on" );    b->color[3]  = BOX_GREEN;
	b->on = 0;

	/* slung load nav status */

	b = p->box[BOX_SLUNGNAV];
	sprintf( b->label,  "SLNav" );
	sprintf( b->slabel, "SLNv" );
	sprintf( b->status0, "OFF" );    b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "Single" ); b->color[1] = BOX_GREEN;
	sprintf( b->status2, "Diff" );   b->color[2] = BOX_GREEN;
	sprintf( b->status3, "RTK" );    b->color[3] = BOX_DARKGREEN;
	b->on = 0;

	/* slung load guidance mode */

	b = p->box[BOX_SLUNGMODE];
	sprintf( b->label,  "SLMode" );
	sprintf( b->slabel, "SLMd" );
	sprintf( b->status0, "Appr" );     b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "FinAppr" );  b->color[1] = BOX_GREEN;
	sprintf( b->status2, "Descent" );  b->color[2] = BOX_GREEN;
	sprintf( b->status3, "FinHold" );  b->color[3] = BOX_GREEN;
	sprintf( b->status4, "FinDesc" );  b->color[4] = BOX_GREEN;
	sprintf( b->status5, "Climb" );    b->color[5] = BOX_GREEN;
	b->on = 0;

	/* Yellow Jacket External 28V bus */

	b = p->box[BOX_YJ_28Ve];
	sprintf( b->label,   "28VBus"  );
	sprintf( b->slabel,  "28E"     );
	sprintf( b->status0, "1-Low"   );  b->color[0] = BOX_YELLOW;
	sprintf( b->status1, "1-High"  );  b->color[1] = BOX_YELLOW;
	sprintf( b->status2, "1-Fail"  );  b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "2-Low"   );  b->color[3] = BOX_YELLOW;
	sprintf( b->status4, "2-High"  );  b->color[4] = BOX_YELLOW;
	sprintf( b->status5, "2-Fail"  );  b->color[5] = BOX_YELLOW;
	sprintf( b->status6, "FAIL"    );  b->color[6] = BOX_RED;
	sprintf( b->status7, "Ok"      );  b->color[7] = BOX_DARKGREEN;
	sprintf( b->status8, "1&2 err" );  b->color[8] = BOX_YELLOW;
	b->on = 0;

	/* Yellow Jacket Relay status */
	b = p->box[BOX_YJ_RELAY2];
	sprintf( b->label,  "Strobe" );
	sprintf( b->slabel, "Strb"  );
	sprintf( b->status0, "Off"  );  b->color[0] = BOX_GREEN;
	sprintf( b->status1, "On"   );  b->color[1] = BOX_DARKGREEN;
	b->on = 0;

	b = p->box[BOX_YJ_RELAY3];
	sprintf( b->label,  "Servo" );
	sprintf( b->slabel, "Srvo"  );
	sprintf( b->status0, "Off"  );  b->color[0] = BOX_RED;
	sprintf( b->status1, "On"   );  b->color[1] = BOX_DARKGREEN;
	b->on = 0;

	b = p->box[BOX_YJ_RELAY4];
	sprintf( b->label,  "BackupBatt" );
	sprintf( b->slabel, "Batt"  );
	sprintf( b->status0, "Off"  );  b->color[0] = BOX_GREEN;
	sprintf( b->status1, "On"   );  b->color[1] = BOX_DARKGREEN;
	b->on = 0;

	b = p->box[BOX_YJ_RELAY1];
	sprintf( b->label,  "Ignition" );
	sprintf( b->slabel, "Ignt"  );
	sprintf( b->status0, "Off"  );   b->color[0] = BOX_RED;
	sprintf( b->status1, "On"   );   b->color[1] = BOX_DARKGREEN;
	b->on = 0;

    /* vicon system */

    b = p->box[BOX_VICON];
    sprintf( b->label,  "VICON" );
    sprintf( b->slabel, "Vicon" );
    sprintf( b->status0, "OFF" );     b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "ON" );      b->color[1] = BOX_GREEN;
    sprintf( b->status2, "NoLock" );  b->color[2] = BOX_RED;
	b->inopFlag = 0;
	b->on = 0;
	b->timeOut = 1;

	/* Hokuyo/Sick Laser */

	b = p->box[BOX_LASER];
	sprintf( b->label,  "LASER" );
	sprintf( b->slabel, "LASER" );
	sprintf( b->status0, "OFF" );    b->color[0] = BOX_RED;
	sprintf( b->status1, "ON" );     b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "SIM" );    b->color[2] = BOX_ORANGE;
	sprintf( b->status3, "SUN");     b->color[3] = BOX_GREEN;
	b->timeOut = -1.0;
	b->on = 0;

	/* SLAM guidance */

	b = p->box[BOX_SLAMGUIDE];
	sprintf( b->label,  "SLAMGuide" );
	sprintf( b->slabel, "SG" );
	sprintf( b->status0, "OFF" );     b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "ENTRY" );   b->color[1] = BOX_GREEN;
	sprintf( b->status2, "COAST" );   b->color[2] = BOX_GREEN;
	sprintf( b->status3, "FRONT" );   b->color[3] = BOX_GREEN;
	sprintf( b->status4, "DESTR" );   b->color[4] = BOX_RED;
	b->timeOut = -1.0;
	b->on = 0;

    b = p->box[BOX_PSPNAV];
    sprintf( b->label,  "PNAV" );
    sprintf( b->slabel, "PNav" );
    sprintf( b->status0, "INIT" );    b->color[0] = BOX_ORANGE;
    sprintf( b->status1, "ON" );      b->color[1] = BOX_GREEN;
	sprintf( b->status2, "SS" );      b->color[2] = BOX_YELLOW;
	b->on = 0;

    /* IMU STATUS */

    b = p->box[BOX_IMU];
    sprintf( b->label,  "IMU" );
    sprintf( b->slabel, "IMU" );
    sprintf( b->status0, "OFF" );     b->color[0] = BOX_RED;
    sprintf( b->status1, "OK" );      b->color[1] = BOX_DARKGREEN;
    sprintf( b->status2, "BAD" );     b->color[2] = BOX_YELLOW;

    /* Fan STATUS */

    b = p->box[BOX_FAN];
    sprintf( b->label,  "FAN" );
    sprintf( b->slabel, "Fan" );
    sprintf( b->status0, "OFF" );     b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "ON" );      b->color[1] = BOX_DARKGREEN;
	b->on = 0;

	/* temperature monitoring */

	b = p->box[BOX_TEMPERATURE];
	sprintf( b->label,  "Temperature" );
	sprintf( b->slabel, "Temp" );
	sprintf( b->status0, "GOOD" );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "LOW"  );  b->color[1] = BOX_YELLOW;
	sprintf( b->status2, "HIGH" );  b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "OVER" );  b->color[3] = BOX_RED;
	b->on = 0;

	/* generator */

	b = p->box[BOX_GENERATOR];
	sprintf( b->label,  "Generator" );
	sprintf( b->slabel, "Gen" );
	sprintf( b->status0, "GOOD" );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "EXT"  );  b->color[1] = BOX_ORANGE;
	sprintf( b->status2, "OFF" );   b->color[2] = BOX_YELLOW;
	sprintf( b->status3, "FAIL" );  b->color[3] = BOX_RED;
	b->on = 0;

	/* Camera Status */
	b = p->box[BOX_CAMERA_STATUS];
	sprintf( b->label,  "Camera" );
	sprintf( b->slabel, "Cam" );
	sprintf( b->status0, "GOOD" );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "MEM"  );  b->color[1] = BOX_YELLOW;
	sprintf( b->status2, "STOP" );  b->color[2] = BOX_DARKGREEN;
	sprintf( b->status3, "FF" );    b->color[3] = BOX_ORANGE;
	sprintf( b->status4, "REC" );   b->color[4] = BOX_RED_NO_WARNING;
	sprintf( b->status5, "REW" );   b->color[5] = BOX_ORANGE;
	sprintf( b->status6, "PLAY" );  b->color[6] = BOX_GREEN;
	b->on = 0;

	/* Payload Status */
	b = p->box[BOX_PAYLOAD_STATUS];
	sprintf( b->label,  "Payload" );
	sprintf( b->slabel, "Pyld" );
	sprintf( b->status0, "GOOD" );  b->color[0] = BOX_DARKGREEN;
	sprintf( b->status1, "BATT"  ); b->color[1] = BOX_YELLOW;
	sprintf( b->status2, "MEM" );   b->color[2] = BOX_RED;
	sprintf( b->status3, "PROT" );  b->color[3] = BOX_YELLOW;
	b->on = 0;

    b = p->box[BOX_ARDUPILOT];
    sprintf( b->label,  "ArduPilot" );
    sprintf( b->slabel, "Ardu" );
    sprintf( b->status0, "OFF" );      b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "GOOD" );     b->color[1] = BOX_DARKGREEN;
    sprintf( b->status2, "OFF" );      b->color[2] = BOX_RED;
    b->timeOut = 2.0;
    b->on = 0;

	b = p->box[BOX_RANGEFINDER];
    sprintf( b->label,  "Rangefinder" );
    sprintf( b->slabel, "LRF" );
	sprintf( b->status0, "OFF"   );  b->color[0] = BOX_RED;
	sprintf( b->status1, "RANGE" );  b->color[1] = BOX_DARKGREEN;
	sprintf( b->status2, "GOOD"  );  b->color[2] = BOX_GREEN;
	sprintf( b->status3, "OUT"   );  b->color[3] = BOX_GREEN; // Outlier - used if displaying second AGL sensor
    b->timeOut = 2.0;
    b->on = 0;
    
	b = p->box[BOX_FLAP];
    sprintf( b->label,  "Flaps" );
    sprintf( b->slabel, "Flaps" );
    sprintf( b->status0, "UP"   );  b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "DOWN" );  b->color[1] = BOX_GREEN;
	b->on = 0;
    
	b = p->box[BOX_GPSREF];
    sprintf( b->label,  "GPSRef" );
    sprintf( b->slabel, "GPSRef" );
    sprintf( b->status0, "OFF"   );  b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "ON"    );  b->color[1] = BOX_DARKGREEN;
    sprintf( b->status2, "CAL"   );  b->color[2] = BOX_ORANGE;
    sprintf( b->status3, "RDY"   );  b->color[3] = BOX_ORANGE;
		b->on = 0;
	b->inopFlag = 0;
    b->timeOut = 10.0;

	b = p->box[BOX_DECK];
    sprintf( b->label,  "Deckpredict" );
    sprintf( b->slabel, "Deck"  );
	sprintf( b->status0, "OFF"  );  b->color[0] = BOX_DARKGREEN;
    sprintf( b->status1, "WAIT" );  b->color[1] = BOX_ORANGE;
	sprintf( b->status2, "FAIL!");  b->color[2] = BOX_ORANGE;
	sprintf( b->status3, "ROUGH");  b->color[3] = BOX_ORANGE;
	sprintf( b->status4, "CALM");   b->color[4] = BOX_GREEN;
	b->on = 0;

	/* BROOM STATUS */

	b = p->box[BOX_BOOM];
	sprintf(b->label, "BOOM");
	sprintf(b->slabel, "TOUCH");
	sprintf(b->status0, "BAD");				b->color[0] = BOX_ORANGE;
	sprintf(b->status1, "OFF");  b->color[1] = BOX_DARKGREEN;
	sprintf(b->status2, "ON");			b->color[2] = BOX_GREEN;
	b->on = 0;
    
}


int initPanel( void ) {

//	struct limitCheck_ref *lc = p->check;

//	char in[BSIZE][BSIZE]= { /* init */
//      "wwwwwwwwwwwwwwwwwwwwwwwwwwwwwww ",
//      "wwwwwwwwwwwwwwwwwwwwwwwwwwwwww b",
//      "ww                            bb",
//      "ww                            bb",
//      "ww    bbbbbbbbbbbbbbbbbbbb    bb",
//      "ww    b                  b    bb",
//      "ww    b wwww  wwww  bbbb b    bb",
//      "ww    b wbbw  wbbw  bwwb b    bb",
//      "ww    b wbbw  wbbw  bwwb b    bb",
//      "ww    b wwww  wwww  bbbb b    bb",
//      "ww    b                  b    bb",
//      "ww    bbbbbbbbbbbbbbbbbbbb    bb",
//      "ww    bbbbbbbbbbbbbbbbbbbb    bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww                            bb",
//      "ww bbb   bb  b   b bbbb b     bb",
//      "ww b  b b  b bb  b b    b     bb",
//      "ww bbb  bbbb b b b bbb  b     bb",
//      "ww b    b  b b  bb b    b     bb",
//      "ww b    b  b b   b bbbb bbbb  bb",
//      "ww                            bb",
//      "w bbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
//      " bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
//    };


	/*addCnslButton( "panel", in );*/
	commandLoad( "panel", commandPanel, "GCS: open panel" );

	initAPanel( &gcs0Panel );

    return 0;

}