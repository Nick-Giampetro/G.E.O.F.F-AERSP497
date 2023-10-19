#ifndef esim_quat_h
#define esim_quat_h

#if defined(__cplusplus)
extern "C"
{
#endif

void normalize_quat( double q[4] );

void matrix_transpose( double in[3][3], double out[3][3] );

void matrix_multiply( double in1[3][3], double in2[3][3], double out[3][3] );

void map_vector( double T[3][3], double vin[3], double vout[3] );
void map_T_vector( double T[3][3], double vin[3], double vout[3] );

/*
 * Given two rotations, e1 and e2, expressed as quaternion rotations,
 * figure out the equivalent single rotation and stuff it into dest.
 *
 * NOTE: This routine is written so that q1 or q2 may be the same
 * as dest (or each other).
 */
void mult_quats( double *q1, double *q2, double *dest );
void mult_quats_norm(double *q1, double *q2, double *dest);

/*
 * A useful function, builds a rotation matrix in Matrix based on
 * given quaternion. Note: quaternion from frame A to B gives dcm
 * from B to A.
 */
void build_rotmatrix( double m[3][3], double q[4] );

/*
 * This function computes a quaternion based on an axis (defined by
 * the given vector) and an angle about which to rotate.  The angle is
 * expressed in radians.  The result is put into the third argument.
 */
void axis_to_quat( double phi, double x, double y, double z, double q[4] );

/* 
 * This function computes the DCM matrix from frame b to a given the 
 * Euler angle rotations from frame a to b
 */
void euler2dcm( double phi, double theta, double psi, double m[3][3] );
void euler2dcmf( float phi, float theta, float psi, float m[4][4] );

/*
 *  Note: dcm from A to B gives psi theta and phi euler anlges
 * from B to A.
 */
void dcm2euler( double m[3][3], 
		double *phi, double *theta, double *psi );

void omega2edot( double p, double q, double r, double e[4], double ed[4] );

void angles2deltae( double pdt, double qdt, double rdt, double e[4], double en[4] );

void saveFloat( double in[3][3], float out[4][4] );

void greatCircle( double lat1, double lon1, double along, double track,
		  double *lat2, double *lon2 );

/** calculates the angle error between two quaternions using small angle approx 
 input: q, quat rotation from A to B
        r, quat rotation from A to C
 output: dest, euler angles from C to B */
void quat_err(double *dest, double *q, double *r);

/** euler to quat */
void euler2quat(double *e, double phi, double theta, double psi);


/** quat 2 euler 
 Note: quat from A to B results in euler angle
 rotations from A to B
 */
void quat2euler(double *q, double *phi, double *theta, double *psi);

void quat2eulerf( float *q, float *phi, float *theta, float *psi );

void map_vector_quat(double q[4], double vin[3], double vout[3]);

/** rotmat1  */
void rotmat1(double m[3][3], double angle);
void rotmat2(double m[3][3], double angle);
void rotmat3(double m[3][3], double angle);

#if defined(__cplusplus)
}
#endif

#endif

