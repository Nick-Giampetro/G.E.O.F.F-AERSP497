/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Thanakorn Khamvilai
 * Email : thanakorn.khamvilai@psu.edu
 *
 * EndCopyright
 ***/

#include "math_utils.h"

mat3x3_t rotation_from_quaternion(const quat_t& q)
{
  mat3x3_t T;
  
  T(0,0) = 1-2*(q(2)*q(2)+q(3)*q(3));
  T(0,1) = 2*(q(1)*q(2)-q(0)*q(3));
  T(0,2) = 2*(q(1)*q(3)+q(0)*q(2));

  T(1,0) = 2*(q(1)*q(2)+q(0)*q(3));
  T(1,1) = 1-2*(q(1)*q(1)+q(3)*q(3));
  T(1,2) = 2*(q(2)*q(3)-q(0)*q(1));

  T(2,0) = 2*(q(1)*q(3)-q(0)*q(2));
  T(2,1) = 2*(q(2)*q(3)-q(0)*q(1));
  T(2,2) = 1-2*(q(1)*q(1)+q(2)*q(2));

  return T;
}

void normalize_quaternion(quat_t& q) 
{
  int i;
  float rmag;

  rmag = sqrt( q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3) );
	if( rmag > 0.01 && rmag < 100.0 ) 
  {
	  for( i = 0; i < 4; i++ ) q(i) /= rmag;
	} 
  else 
  {
		/* quat is messed up */
		q(0) = 1;
		q(1) = 0;
		q(2) = 0;
		q(3) = 0;
	}
}

void quat2euler( const quat_t& q, float& phi, float& theta, float& psi ) {

	// mat3x3_t m;

	// float q0_x_q1 = q(0)*q(1);
	// float q0_x_q2 = q(0)*q(2);
	// float q1_x_q3 = q(1)*q(3);
	// float q2_x_q3 = q(2)*q(3);

	// m(2,0) = 2.0*( q1_x_q3 - q0_x_q2 );

	// if( abs( m(2,0) ) < 0.9999 ) {
	// 	float q0_x_q3 = q(0)*q(3);
	// 	float q1_x_q1 = q(1)*q(1);
	// 	float q1_x_q2 = q(1)*q(2);
	// 	float q2_x_q2 = q(2)*q(2);
	// 	float q3_x_q3 = q(3)*q(3);
	// 	m(0,0) = 1.0 - 2.0*( q2_x_q2 + q3_x_q3 );
	// 	m(1,0) =       2.0*( q1_x_q2 + q0_x_q3 );
	// 	m(2,1) =       2.0*( q2_x_q3 + q0_x_q1 );
	// 	m(2,2) = 1.0 - 2.0*( q1_x_q1 + q2_x_q2 );
	// 	phi   = atan2( m(2,1), m(2,2) );
	// 	theta = -asin( constrain( m(2,0), -1.0, 1.0 ) );
	// 	psi   = atan2( m(1,0), m(0,0) );
	// } else {
	// 	m(0,2) = 2.0*( q1_x_q3 + q0_x_q2 );
	// 	m(1,2) = 2.0*( q2_x_q3 - q0_x_q1 );
	// 	phi   = 0.0;
	// 	if( m(2,0) < 0.0 ) {
	// 		theta =  M_PI*0.5;
	// 		psi   = atan2( m(1,2), m(0,2) );
	// 	} else {
	// 		theta = -M_PI*0.5;
	// 		psi   = atan2( -m(1,2), -m(0,2) );
	// 	}
	// }

  /* roll (x-axis rotation) */
  float sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
  float cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
  phi = atan2(sinr_cosp, cosr_cosp);

  /* pitch (y-axis rotation) */
  float sinp = 2 * (q(0) * q(2) - q(3) * q(1));
  if (abs(sinp) >= 1)
      theta = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      theta = asin(sinp);

  /* yaw (z-axis rotation) */
  float siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
  float cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
  psi = atan2(siny_cosp, cosy_cosp);
}

