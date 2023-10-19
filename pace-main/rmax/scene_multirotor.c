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

/* comment to force recompile */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "esim/util.h"
#include "esim/quat.h"
#include "rmax/scene.h"
#include "rmax/generic_ref.h"
#include "rmax/scene_multirotor.h"
#include "rmax/motion_ref.h"
#include "rmax/onboard_ref.h"
#include "rmax/gear_ref.h"

void initMultirotor( void ) {

	struct multirotorModel_ref *m     = &multirotorModel;
	struct rotorMultirotor_ref *ro    = &rotorMultirotor;
	struct pwmMultirotor_ref   *multi = &pwmMultirotor;

	int i, j;
	double angle, propz, dx[3];
	float armRadius, motorRadius;

	for( i=0; i<multi->numberOfRotors; ++i ) {

		/* determine phase angle */
        if( multi->rotorsPaired ) {
            ro->phase[i] = C_PI*2.0*( 0.5 + i )/(0.5*multi->numberOfRotors) + multi->phaseAngle*C_DEG2RAD; /* 0 out the nose, 90deg on right wing */
        } else {
            ro->phase[i] = C_PI*2.0*( 0.5 + i )/(    multi->numberOfRotors) + multi->phaseAngle*C_DEG2RAD; /* 0 out the nose, 90deg on right wing */
        }

		/* find dcm for each rotor */
		/* note chose benefitial yaw authority choice for motor twists */
		ro->twistPer[i] = -ro->motorTwist*multi->direction[i];

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
		
	/* shadow display list */
	if( !glIsList( m->shadow_dl ) )
		m->shadow_dl = glGenLists( 1 );
	glNewList( m->shadow_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	glDisable( GL_LIGHTING );
	//glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, m->shadowColor );
	glColor4fv( m->shadowColor );

	/* props */
	for( i=0; i<multi->numberOfRotors; ++i ) {
		glPushMatrix();
		glTranslatef( (float)(ro->x[i]), (float)(ro->y[i]), 0 );
		glRotatef( (float)(ro->phase[i]*C_RAD2DEG), 0, 0, 1 );
		glScalef( (float)(ro->r*cos( ro->dihedral )), (float)(ro->r*cos( ro->twistPer[i] )), 1 );
		glBegin( GL_POLYGON );
		for( angle = 0; angle < 2*C_PI; angle += C_PI/36 ) {
			glVertex3f( (float)sin( angle ),  (float)cos( angle ), 0 );
		}
		glEnd();
		glPopMatrix();
	}

	/* "fuselage" */
    glPushMatrix();
	glScalef( (float)( ro->armRadius - ro->r )*m->fuseSize[0], (float)( ro->armRadius - ro->r )*m->fuseSize[1], 0 );
	glBegin( GL_POLYGON );
	glVertex3f( +1.4f, 0, -1 );  glVertex3f( -1, -1, -1 );  glVertex3f( -1, +1, -1 );
	glEnd();
	glPopMatrix();

	glEnable( GL_LIGHTING );
	glEndList();

	/* main display list */
	if( !glIsList( m->body_dl ) )
		m->body_dl = glGenLists( 1 );
	glNewList( m->body_dl, GL_COMPILE );

	/* "fuselage" */
    glPushMatrix();
	glScalef( (float)( ro->armRadius - ro->r )*m->fuseSize[0], (float)( ro->armRadius - ro->r )*m->fuseSize[1], (float)( ro->armRadius - ro->r )*m->fuseSize[2] );
	glBegin( GL_QUADS );
	/* back */
	glNormal3f( -1, 0, 0 );
	glVertex3f( -1, +1, -1 );  glVertex3f( -1, -1, -1 );  glVertex3f( -1, -1, +1 );  glVertex3f( -1, +1, +1 );
	/* left */
	glNormal3f( 0.38f, -0.92f, 0 );
	glVertex3f( -1, -1, -1 );  glVertex3f( +1.4f, 0, -1 );  glVertex3f( +1.4f, 0, +1 );  glVertex3f( -1, -1, +1 );
	/* right */
	glNormal3f( 0.38f, +0.92f, 0 );
	glVertex3f( +1.4f, 0, -1 );  glVertex3f( -1, +1, -1 );  glVertex3f( -1, +1, +1 );  glVertex3f( +1.4f, 0, +1 );
	glEnd();
	glBegin( GL_TRIANGLES );
	/* top */
	glNormal3f( -0.95f, 0, -0.2f );
	glVertex3f( -0.5f, 0, -4 );  glVertex3f( -1, -1, -1 );  glVertex3f( -1, +1, -1 );
	glNormal3f( 0.57f, -0.57f, -0.57f );
	glVertex3f( -0.5f, 0, -4 );   glVertex3f( +1.4f, 0, -1 );  glVertex3f( -1, -1, -1 );  
	glNormal3f( 0.57f, +0.57f, -0.57f );
	glVertex3f( -0.5f, 0, -4 );  glVertex3f( -1, +1, -1 );    glVertex3f( +1.4f, 0, -1 );  
	/* bottom */
	glNormal3f( 0, 0, +1 );
	glVertex3f( -1, +1, +1 );  glVertex3f( -1, -1, +1 );  glVertex3f( +1.4f, 0, +1 );
	glEnd();
	glPopMatrix();

	/* arms */
	armRadius   = m->armRadiusMultiplier  *(float)(ro->r);
	motorRadius = m->motorRadiusMultiplier*(float)(ro->r);
	for( i=0; i<multi->numberOfRotors; ++i ) {
		glPushMatrix();
		glTranslatef( 0, 0, (float)(ro->armz) );
		glRotatef( (float)(ro->phase[i]*C_RAD2DEG),     0, 0, 1 );
		glRotatef( (float)(ro->dihedral*C_RAD2DEG), 0, 1, 0 );
		glRotatef( (float)(ro->twistPer[i]*C_RAD2DEG),     1, 0, 0 );
		/* arm strut */
		glBegin( GL_QUAD_STRIP );
		for( angle = 0; angle < 2*C_PI; angle += C_PI/18 ) {
			glNormal3f( 0, (float)sin(angle), (float)cos( angle ) );  
			glVertex3f(                      0, armRadius*(float)sin(angle), armRadius*(float)cos(angle) ); 
			glVertex3f( (float)(ro->armRadius), armRadius*(float)sin(angle), armRadius*(float)cos(angle) ); 
		}
		glNormal3f( 0, 0, 1 );  glVertex3f( 0, 0, armRadius ); glVertex3f( (float)(ro->armRadius), 0, armRadius );
		glEnd();
		glPopMatrix();
		/* motor */
		glPushMatrix();
		glTranslatef( 0, 0, (float)(ro->armz) );
		glRotatef( (float)(ro->phase[i]*C_RAD2DEG),     0, 0, 1 );
		glRotatef( (float)(ro->dihedral*C_RAD2DEG), 0, 1, 0 );
		glRotatef( (float)(ro->twistPer[i]*C_RAD2DEG),     1, 0, 0 );
		glTranslatef( (float)(ro->armRadius), 0, 0 );
		glBegin( GL_QUAD_STRIP );
		for( angle = 0; angle < 2*C_PI; angle += C_PI/18 ) {
			glNormal3f( (float)sin(angle), (float)cos( angle ), 0 );  
			glVertex3f( motorRadius*(float)sin(angle), motorRadius*(float)cos(angle), -motorRadius ); 
			glVertex3f( motorRadius*(float)sin(angle), motorRadius*(float)cos(angle), +motorRadius ); 
		}
		glNormal3f( 0, 1, 0 );  glVertex3f( 0, motorRadius, -motorRadius ); glVertex3f( 0, motorRadius, +motorRadius );
		glEnd();
		glBegin( GL_POLYGON );
		glNormal3f( 0, 0, -1 );
		for( angle = 0; angle < 2*C_PI; angle += C_PI/36 ) {
			glVertex3f( (float)sin( angle )*motorRadius,  (float)cos( angle )*motorRadius, -motorRadius );
		}
		glEnd();
		glBegin( GL_POLYGON );
		glNormal3f( 0, 0, +1 );
		for( angle = 0; angle < 2*C_PI; angle += C_PI/36 ) {
			glVertex3f( (float)sin( angle )*motorRadius,  -(float)cos( angle )*motorRadius, +motorRadius );
		}
		glEnd();
		glPopMatrix();
	}

	/* propeller disks */
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, m->propColor );
	glDepthMask( 0 );
	for( i=0; i<multi->numberOfRotors; ++i ) {
		glPushMatrix();
		glTranslatef( (float)(ro->x[i]), (float)(ro->y[i]), (float)(ro->z[i]) );
		glRotatef( (float)(ro->phase[i]*C_RAD2DEG),     0, 0, 1 );
		glRotatef( (float)(ro->dihedral*C_RAD2DEG), 0, 1, 0 );
		glRotatef( (float)(ro->twistPer[i]*C_RAD2DEG),     1, 0, 0 );
		glScalef( (float)(ro->r), (float)(ro->r), 1 );
		glBegin( GL_POLYGON );
		glNormal3f( 0, 0, -1 );
		for( angle = 0; angle < 2*C_PI; angle += C_PI/36 ) {
			glVertex3f( (float)sin( angle ),  (float)cos( angle ), 0 );
		}
		glEnd();
		glBegin( GL_POLYGON );
		glNormal3f( 0, 0, 1 );
		for( angle = 0; angle < 2*C_PI; angle += C_PI/36 ) {
			glVertex3f( (float)sin( angle ),  -(float)cos( angle ), 0 );
		}
		glEnd();
		glPopMatrix();
	}
	glDepthMask( 1 );

	glEndList();

} /* end initMultirotor function */


void drawMultirotor( const double latitude,
		  const double longitude,
		  const double altitude,
		  const double terrainAlt,
		  const double eyeLat,
		  const double eyeLon,
		  const double eyeAlt,
		  const double cosDatumLat,
		  const double fovy,
		  const float dcm[4][4],
		  float sunPosition[4],
		  int mode, int winh ) {

	struct multirotorModel_ref *m = &multirotorModel;
	struct pwmMultirotor_ref   *multi = (&onboard)->actuators->pwmMulti;
	struct params_gear_ref     *gear  = &params_gear;

	float phi, theta, psi, distance, enlarge;

	int i;

	phi   = (float)atan2( dcm[1][2], dcm[2][2] );
    theta = -(float)asin( LIMIT( dcm[0][2], -1.0, 1.0 ) );
    psi   = (float)atan2( dcm[0][1], dcm[0][0] );

	if( mode == 1 ) {
		enlarge = m->scaleSize;
	} else {
		if( fovy > 0 ) {
			distance = (float)sqrt( SQ( ( latitude - eyeLat )*C_NM2FT*60.0 ) +
			SQ( hmodDeg( longitude - eyeLon )*C_NM2FT*60.0*cosDatumLat ) +
			SQ( -altitude + eyeAlt ) );
			enlarge = (float)(MAX( m->scaleSize, distance*m->minSize/10*fovy/winh ));
		} else {
			enlarge = (float)(MAX( m->scaleSize, -m->minSize/10*fovy/winh ));
		}
	}

	if( m->drawShadow ) {
		glPushMatrix();
		glTranslatef( (float)(( latitude - eyeLat )*C_NM2FT*60.0 /*-
			(altitude-terrainAlt)*sunPosition[0]*/ ),
			(float)(hmodDeg( longitude - eyeLon )*C_NM2FT*60.0*cosDatumLat /* -
			(altitude-terrainAlt)*sunPosition[1]*/ ), (float)(-terrainAlt + eyeAlt) );
		glRotatef( psi*CF_RAD2DEG, 0.0, 0.0, 1.0 );
		glScalef( ABS( (float)cos( theta )), ABS( (float)cos( phi )), 1.0 );
		glDepthMask( 0 );
		glCallList( m->shadow_dl );
		glDepthMask( 1 );
		glPopMatrix();
	}

	/* draw main body */
	glPushMatrix();
	glTranslatef( (float)(( latitude - eyeLat )*C_NM2FT*60.0),
				  (float)(hmodDeg( longitude - eyeLon )*C_NM2FT*60.0*cosDatumLat),
				  (float)(-altitude + eyeAlt) );
	if( mode == 1 )  { // switch color for Truth model
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, m->bodyColor );
	} else {
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, m->body2Color );
	}
	glScalef( enlarge, enlarge, enlarge );
	glMultMatrixf( &dcm[0][0] );
	glShadeModel( GL_SMOOTH );
	glCallList( m->body_dl );

	/* landing gear */
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, m->gearColor );
	glBegin( GL_LINES );
	glNormal3f( 0, 0, -1 );
	for( i=0; i<NCONTACTPOINTS; i++ ) {
		if( mode == 1 ) {
			if( i != 6 ) {
				glVertex3f( 0, 0, 0 );  glVertex3f( (float)gear->xw[i], (float)gear->yw[i], (float)( gear->zw[i] - gear->deflection[i] ) );
			}
		} else {
			if( i != 6 ) {
				glVertex3f( 0, 0, 0 );  glVertex3f( (float)gear->xw[i], (float)gear->yw[i], (float)gear->zw[i] );
			}
		}
	}
	glEnd();

	if( 1 == mode && 1 == m->drawThrust ) {
		struct rotorMultirotor_ref *ro = &rotorMultirotor;
		if( ro->motorsArmed ) {
			int i;
			double angle;

			/* propeller thrust indicators */
			glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, m->thrustColor );
			glLineWidth( m->thrustLineWidth );
			for( i=0; i<multi->numberOfRotors; ++i ) {
				glPushMatrix();
				glTranslatef( (float)(ro->x[i]), (float)(ro->y[i]), (float)(ro->z[i]) );
				glRotatef( (float)(ro->phase[i]   *C_RAD2DEG), 0, 0, 1 );
				glRotatef( (float)(ro->dihedral   *C_RAD2DEG), 0, 1, 0 );
				glRotatef( (float)(ro->twistPer[i]*C_RAD2DEG), 1, 0, 0 );
				glScalef( (float)(ro->r), (float)(ro->r), 1 );
				glBegin( GL_LINE_STRIP );
				glNormal3f( 0, 0, -1 );
				for( angle = 0; angle <= 2*C_PI*ro->pwm[i]; angle += C_PI/36 ) {
					glVertex3f( -(float)cos( angle ),  -(float)sin( angle*multi->direction[i] ), 0 );
				}
				glEnd();
				glPopMatrix();
			}
			glLineWidth( 1 );
		}
	}

	glPopMatrix();

	//-----------------------------------------------------------------
	 // Draw flat bottom
	//if( 1 == m->showFlatBottom ) {

	//	//struct vehicleContact_ref  *vehc = &vehicleContact;	

	//	glDisable(GL_LIGHTING); 
	//	glDisable(GL_CULL_FACE); //allows you to draw both sides of object
 //   
	//	glPushMatrix();
	//	glTranslatef( (float)(( latitude - eyeLat )*C_NM2FT*60.0),
	//				  (float)(hmodDeg( longitude - eyeLon )*C_NM2FT*60.0*cosDatumLat),
	//				  (float)(-altitude + eyeAlt) );
	//	glMultMatrixf( &dcm[0][0] );
	//	glColor4f( m->blackColor[0], m->blackColor[1], m->blackColor[2], 0.7f );
	//	glBegin(GL_QUADS);
	//	glVertex3f(  vehc->b1,  vehc->b2, vehc->b3);
	//	glVertex3f( -vehc->b1,  vehc->b2, vehc->b3);
	//	glVertex3f( -vehc->b1, -vehc->b2, vehc->b3);
	//	glVertex3f(  vehc->b1, -vehc->b2, vehc->b3);
	//	glEnd();
	//
	//	glPopMatrix();
	//	glEnable(GL_CULL_FACE);
	//	glEnable(GL_LIGHTING);
	//}

} /* end drawMultirotor function */


