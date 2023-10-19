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
/***
 * $Id: matrix.cpp,v 1.5 2007-05-22 22:16:31 ejohnson Exp $
 * generic matrix manipulation routines, matrices are stored as arrays of doubles
 ***/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "esim/util.h"
#include "rmax/matrix.h"


void lowpass( int init, double input, double dt, double tc, double *state ) {

    if( init || tc <= dt ) {

        state[0] = input;

    } else {

        if( dt > 0.0 ) {
            state[0] += (input - state[0] )*dt/tc;
        }

    }

}

double notch( int init, double input, double dt,
              double frequency, double spread, double attenuation, double *state ) {

    /* spread: ->0 gives sharper cutoff, ->infinity gives flat cutoff */
    /* attenuation: ->0 does nothing, ->one gives infinite gain reduction at freq */

    double output, stateDot[2], stateDot2[2];

    if( init || frequency == 0.0 ) {

        state[0] = 0.0;
        state[1] = 0.0;
        output = input;

    } else {

        if( dt > 0.0 ) {

            /* to prevent numerical problems */
            spread = LIMIT( spread, 0, 0.5/( dt*frequency ) );
            attenuation = LIMIT( attenuation, 0, 1 );

            stateDot[1] = state[0];
            stateDot[0] = -frequency*spread*2*state[0] - SQ( frequency )*state[1] + input;

            state[0] += stateDot[0]*dt;
            state[1] += stateDot[1]*dt;

            stateDot2[1] = state[0];
            stateDot2[0] = -frequency*spread*2*state[0] - SQ( frequency )*state[1] + input;

            state[0] += ( stateDot2[0] - stateDot[0] )*0.5*dt;
            state[1] += ( stateDot2[1] - stateDot[1] )*0.5*dt;

        }

        output = input - frequency*spread*attenuation*2*state[0];

    }

    return output;

}

/*
 * subroutine to initialize or update delayed states in buffer.
 * init:   flag to initialize delayed states
 * in:     value of state at current time
 * buffer: pointer to vector of delayed states
 * bufferSize:   size of buffer available
 */
void time_delay_in( int init, double in, double *buffer, int bufferSize ) {

	if( init ) {
	    int i;

		for( i=0; i<bufferSize; i++ )
			buffer[i] = in;

		return;
	}

	/* update the buffer */
	memmove( buffer+1, buffer, sizeof( double )*( bufferSize-1 ) );
	buffer[0] = in;

}

/*
 * subroutine to retrieve delayed states from buffer.
 * iDelay: "frames" of delay (need not be an integer), units are number of calls to this function
 * buffer: pointer to vector of delayed states
 * bufferSize:   size of buffer available
 */
double time_delay_out( double iDelay, double *buffer, int bufferSize ) {

	double out;

	/* get output ready */
	if( iDelay >= bufferSize-1 ) {
		out = buffer[bufferSize-1]; /* best we can do... */
	} else { /* more than one frame of delay, less than max */
		int start;
		double frac;
		start = (int)floor(iDelay); 
		frac  = iDelay - floor(iDelay);
		out = buffer[start]*( 1 - frac ) + buffer[start+1]*frac;
	}

	return out;

}

/* this combines both of the above, convenient sometimes to be more compact */

double time_delay( int init, double in, double iDelay, double *buffer, int bufferSize ) {

	double out;

	time_delay_in( init, in, buffer, bufferSize );
	out = time_delay_out( iDelay, buffer, bufferSize );

	return out;

}

/* LEGACY, old way of doing it - try to use the above for new stuff
/*
 * subroutine to initialize or retrieve delayed states from
 * buffer.
 * init:   flag to initialize delayed states
 * in:     value of state at current time
 * buffer: pointer to vector of delayed states
 * index:  pointer to index in the buffer of desired delayed state
 * size:   number of states to delay
 */
double latency( int init, double in, double *buffer,
			    int *index, int size ) {

	double out;

	if( init ) {
		int i;

		*index = 0;
		for( i=0; i<size; i++ )
			buffer[i] = in;
        return in;
	}

	if( size == 0 ) return in;

	out = buffer[*index];
    buffer[*index] = in;
	(*index)++;
	if( *index >= size )
		*index = 0;

	return out;

}

/*  subroutine to initialize a two dimensional array to an arbitrary value
inputs are:  2D array, number of rows, number of columns, and
initial vaule.
*/

#define QUITIF(x) if(x) printf("matrix: dim problem\n")


void mat_init( double *in, int rows, int cols, double init_val ) {

	int i;
	for( i=0; i<rows*cols; i++ )
		in[i] = init_val;

}

void mat_init_sub( double *in, int rows, int cols, int size, double init_val ) {

	int i, j;
	for( i=0; i<rows; i++ )
		for( j=0; j<cols; j++ )
			in[i*size+j] = init_val;

}

/*  subroutine to initialize a two dimensional diagonal array to an arbitrary value
inputs are:  2D array, number of rows, number of columns, and
initial vaule.
*/

void mat_diag_init( double *in, int rows, int cols, double init_val ) {

	int i, j;
	for( i=0; i<rows; i++ )
        for(j=0;j<cols;j++)
            if(i==j)
                in[i*rows+j] = init_val;
			else
				in[i*rows+j] = 0;

}




/*  subroutine to multiply two matrices
C = A*B
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void mat_mult( double *A, int na, int ma,
 			   double *B, int nb, int mb,
			   double *C ) {

	int i, j, k;

	QUITIF( ma != nb );

	mat_init( C, na, mb, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < mb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*mb+j] += A[i*ma+k]*B[k*mb+j];

}

/*  subroutine to multiply subset of two large matrices
C = A*B
inputs are:  A, na (rows in A), ma (cols in A), la (number of cols defined for A)
B, nb (rows in B), mb (cols in B), lb (number of cols defined for B), 
C and lc (number of cols defined for C)
*/
void mat_mult_sub( double *A, int na, int ma, int la,
 			       double *B, int nb, int mb, int lb,
			       double *C, int lc) {

	int i, j, k;

	QUITIF( ma != nb );

	mat_init_sub( C, na, mb, lc, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < mb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*lc+j] += A[i*la+k]*B[k*lb+j];
}


/*  subroutine to multiply a matrix another matrix transposed
C = A*B'
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void mat_mult_T( double *A, int na, int ma,
				double *B, int nb, int mb, double *C ) {

	int i, j, k;

	QUITIF( ma != mb );

	mat_init( C, na, nb, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < nb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*nb+j] += A[i*ma+k]*B[j*ma+k];

}


/*  subroutine to multiply a matrix another matrix transposed
C = A*B'
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void mat_mult_T_sub( double *A, int na, int ma, int la,
				double *B, int nb, int mb, int lb, double *C, int lc ) {

	int i, j, k;

	QUITIF( ma != mb );

	mat_init_sub( C, na, nb, lc, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < nb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*lc+j] += A[i*la+k]*B[j*lb+k];

}


/*  subroutine to multiply a matrix transposed by another matrix
C = (A')*B
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void mat_T_mult( double *A, int na, int ma,
				 double *B, int nb, int mb, double *C ) {

	int i, j, k;

	QUITIF( na != nb );

	mat_init( C, ma, mb, 0.0 );

	for( i = 0; i < ma; i++ )
		for( j = 0; j < mb; j++ )
			for( k = 0; k < na; k++ )
				C[i*mb+j] += A[k*ma+i]*B[k*mb+j];

}


/*  subroutine to multiply a submatrix transposed by another submatrix
C = (A')*B
inputs are:  A, na (rows in A), ma (cols in A), la (total rows in container)
B, nb (rows in B), mb (cols in B), lb (total rows in container)
and C, lc (total rows in container)
*/

void mat_T_mult_sub( double *A, int na, int ma, int la,
                     double *B, int nb, int mb, int lb, 
                     double *C,                 int lc ) {

    int i, j, k;

    QUITIF( na != nb );

    mat_init_sub( C, ma, mb, lc, 0.0 );

    for( i = 0; i < ma; i++ )
        for( j = 0; j < mb; j++ )
            for( k = 0; k < na; k++ )
                C[i*lc+j] += A[k*la+i]*B[k*lb+j];

}


/*  subroutine to add two matrices
C = A + B
inputs are:  A, na (rows in A), ma (cols in A)
B, and C
*/

void mat_add( double *A, int na, int ma, double *B, double *C ) {

	int i, j;

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[i*ma+j] = A[i*ma+j] + B[i*ma+j];

}


/*  subroutine to subtract a matrix from the other
C = A - B
inputs are:  A, na (rows in A), ma (cols in A)
B, and C
*/

void mat_sub( double *A, int na, int ma, double *B, double *C ) {

	int i, j;

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[i*ma+j] = A[i*ma+j] - B[i*ma+j];

}


/*  subroutine to multiply two matrices element by element
C = A.*B
inputs are:  A, na (rows in A), ma (cols in A)
B and C
*/

void mat_elem_mult( double *A, int na, int ma, double *B, double *C ) {

	int i;
	for( i = 0; i < na*ma; i++ ) {
		C[i] = A[i]*B[i];
	}

}


/*  subroutine to multiply a matrix by a scalar
C = A*b
inputs are:  A, na (rows in A), ma (cols in A)
b (a scalar) and C
*/

void mat_scalar_mult( double *A, int na, int ma, double b, double *C ) {

	int i;
	for(i=0;i<na*ma;i++)
	{
		C[i] = A[i]*b;
	}

}


/*  subroutine to transpose a matrix
C = A'
inputs are:  A, na (rows in A), ma (cols in A), C
*/

void mat_transpose( double *A, int na, int ma, double *C ) {

	int i, j;

	mat_init( C, ma, na, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[j*na+i] = A[i*ma+j];

}


/*  subroutine to compute the trace of a square matrix
ie.  tr(A)
inputs are:  A, na (rows and also columns in A)
*/

double mat_trace( double *A, int na ) {

	int i;
	double trace_A = 0.0;  /** initialize to zero **/

	for( i = 0; i < na; i++ )
		trace_A += A[i*na + i];

	return trace_A;

}


/*  subroutine to copy the contents of one matrix to another matrix
B = A
inputs are:  A, na (rows in A), ma (cols in A)
and B
*/

void mat_copy( double *A, int na, int ma, double *B ) {

	memcpy( B, A, sizeof(double)*na*ma );

}


static void lu_decomp( double *A, int n, int *indx, double *d ) {

	double vv[MAXSIZE];  /* vv[n] stores implicit scaling of each row */
	register int       i, imax=0, j, k;
	register double big, dum, sum, temp;

	/*if( n > MAXSIZE ) {
	fprintf( stderr, "lu_decomp: n too large\n" );
	return ( -1 );
    }*/

	*(d) = 1.0;

	for( i = 0; i < n; i++ ) {
		big = 0.0;
		for( j = 0; j < n; j++ )
			if( (temp = ABS(A[i*n + j])) > big )
				big = temp;
				/*if ( big == 0.0 ) {
				fprintf( stderr, "lu_decomp: matrix is singular\n" );
				return ( -1 );
	            }*/
			vv[i] = 1.0/big;
	}

	for( j = 0; j < n; j++ ) {
		for( i = 0; i < j; i++ ) {
			sum = A[i*n + j];
			for( k = 0; k < i; k++ )
				sum -= A[i*n + k]*A[k*n + j];
			A[i*n + j] = sum;
		}
		big = 0.0;
		for( i = j; i < n; i++ ) {
			sum = A[i*n + j];
			for( k = 0; k < j; k++ )
				sum -= A[i*n + k]*A[k*n + j];
			A[i*n + j] = sum;
			if( (dum = vv[i]*ABS(sum)) >= big ) {
				big  = dum;
				imax = i;
			}
		}
		if( j != imax ) {
			for( k = 0; k < n; k++ ) {
				dum           = A[imax*n + k];
				A[imax*n + k] = A[j*n + k];
				A[j*n + k]    = dum;
			}
			*(d) = -(*(d));
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if( A[j*n + j] == 0.0 )
			A[j*n + j] = 1.0e-12;
		if( j != n-1 ) {
			dum = 1.0/A[j*n + j];
			for ( i = j+1; i < n; i++ ) A[i*n + j] *= dum;
		}
	}

}

/** For flexible sized-matrices with allocated size up to maxn */
static void lu_decomp_flexsize( double *A, int n, int maxn, int *indx, double *d ) {

	double vv[MAXFLEXSIZE];  /* vv[n] stores implicit scaling of each row */
	register int       i, imax=0, j, k;
	register double big, dum, sum, temp;

	/*if( n > MAXSIZE ) {
	fprintf( stderr, "lu_decomp: n too large\n" );
	return ( -1 );
    }*/

	*(d) = 1.0;

	for( i = 0; i < n; i++ ) {
		big = 0.0;
		for( j = 0; j < n; j++ )
			if( (temp = ABS(A[i*maxn + j])) > big )
				big = temp;
				/*if ( big == 0.0 ) {
				fprintf( stderr, "lu_decomp: matrix is singular\n" );
				return ( -1 );
	            }*/
			vv[i] = 1.0/big;
	}

	for( j = 0; j < n; j++ ) {
		for( i = 0; i < j; i++ ) {
			sum = A[i*maxn + j];
			for( k = 0; k < i; k++ )
				sum -= A[i*maxn + k]*A[k*maxn + j];
			A[i*maxn + j] = sum;
		}
		big = 0.0;
		for( i = j; i < n; i++ ) {
			sum = A[i*maxn + j];
			for( k = 0; k < j; k++ )
				sum -= A[i*maxn + k]*A[k*maxn + j];
			A[i*maxn + j] = sum;
			if( (dum = vv[i]*ABS(sum)) >= big ) {
				big  = dum;
				imax = i;
			}
		}
		if( j != imax ) {
			for( k = 0; k < n; k++ ) {
				dum           = A[imax*maxn + k];
				A[imax*maxn + k] = A[j*maxn + k];
				A[j*maxn + k]    = dum;
			}
			*(d) = -(*(d));
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if( A[j*maxn + j] == 0.0 )
			A[j*maxn + j] = 1.0e-12;
		if( j != n-1 ) {
			dum = 1.0/A[j*maxn + j];
			for ( i = j+1; i < n; i++ ) A[i*maxn + j] *= dum;
		}
	}

}

/******************************************************************************
 */

static void lu_back_sub( double *A, int n, int *indx,  double *b ) {

	register int    i, ii, ip, j;
	register double sum;

	ii = -1;

	for( i = 0; i < n; i++ ) {
		ip  = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if( ii != -1 )
			for( j = ii; j < i; j++ )
				sum -= A[i*n + j]*b[j];
			else if ( sum )
				ii = i;
			b[i] = sum;
	}

	for( i = n-1; i > -1; i-- ) {
		sum = b[i];
		for ( j = i+1; j < n; j++ )
			sum -= A[i*n + j]*b[j];
		b[i] = sum/A[i*n + i];
	}

}

/** For flexible sized-matrices with allocated size up to maxn */
static void lu_back_sub_flexsize( double *A, int n, int maxn, int *indx,  double *b ) {

	register int    i, ii, ip, j;
	register double sum;

	ii = -1;

	for( i = 0; i < n; i++ ) {
		ip  = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if( ii != -1 )
			for( j = ii; j < i; j++ )
				sum -= A[i*maxn + j]*b[j];
			else if ( sum )
				ii = i;
			b[i] = sum;
	}

	for( i = n-1; i > -1; i-- ) {
		sum = b[i];
		for ( j = i+1; j < n; j++ )
			sum -= A[i*maxn + j]*b[j];
		b[i] = sum/A[i*maxn + i];
	}

}

/******************************************************************************
*/

void mat_invert( double *A, int n, double *Ainv /*, double *det*/ ) {

	int indx[MAXSIZE];   /* index is dimension n */
	double b[MAXSIZE],   /* b is dimension n     */
		Acopy[MAXSIZE*MAXSIZE];   /* Acopy is n by n      */
	register int i, j;

	/* just to remove *det from arguments */
	double determinant, *det;
	det = &determinant;

	/*QUITIF( n > MAXSIZE );*/

	for( i = 0; i < n; i++ ) {
		for( j = 0; j < n; j++ )
			Acopy[i*n + j] = A[i*n + j];
	}

	lu_decomp( Acopy, n, indx, det );

	for( j = 0; j < n; j++ ) {
		*(det) *= Acopy[j*n + j];
		for( i = 0; i < n; i++ )
			b[i] = 0.0;
		b[j] = 1.0;
		lu_back_sub( Acopy, n, indx, b );
		for( i = 0; i < n; i++ )
			Ainv[i*n + j] = b[i];
	}

}

/** For flexible sized-matrices with allocated size up to maxn */
void mat_invert_flexsize( double *A, int n, int maxn, double *Ainv /*, double *det*/ ) {

	int indx[MAXFLEXSIZE];   /* index is dimension n */
	double b[MAXFLEXSIZE],   /* b is dimension n     */
		Acopy[MAXFLEXSIZE*MAXFLEXSIZE];   /* Acopy is n by n      */
	register int i, j;

	/* just to remove *det from arguments */
	double determinant, *det;
	det = &determinant;

	/*QUITIF( n > MAXSIZE );*/

	for( i = 0; i < n; i++ ) {
		for( j = 0; j < n; j++ )
			Acopy[i*maxn + j] = A[i*maxn + j];
	}

	lu_decomp_flexsize( Acopy, n, maxn, indx, det );

	for( j = 0; j < n; j++ ) {
		*(det) *= Acopy[j*maxn + j];
		for( i = 0; i < n; i++ )
			b[i] = 0.0;
		b[j] = 1.0;
		lu_back_sub_flexsize( Acopy, n, maxn, indx, b );
		for( i = 0; i < n; i++ )
			Ainv[i*maxn + j] = b[i];
	}

}


/* floating point versions */


/*  subroutine to initialize a two dimensional array to an arbitrary value
inputs are:  2D array, number of rows, number of columns, and
initial vaule.
*/

void matf_init( float *in, int rows, int cols, float init_val ) {

	int i, j;

	for( i = 0; i < rows; i++ )
		for( j = 0; j < cols; j++ )
			in[i*cols+j] = init_val;

}


/*  subroutine to multiply two matrices
C = A*B
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void matf_mult( float *A, int na, int ma,
               float *B, int nb, int mb,
               float *C ) {

	int i, j, k;

	/*QUITIF( ma != nb );*/

	matf_init( C, na, mb, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < mb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*mb+j] += A[i*ma+k]*B[k*mb+j];

}


/*  subroutine to multiply a matrix another matrix transposed
C = A*B'
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void matf_mult_T( float *A, int na, int ma,
                 float *B, int nb, int mb, float *C ) {

	int i, j, k;

	/*QUITIF( ma != mb );*/

	matf_init( C, na, nb, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < nb; j++ )
			for( k = 0; k < ma; k++ )
				C[i*nb+j] += A[i*ma+k]*B[j*ma+k];

}


/*  subroutine to multiply a matrix transposed by another matrix
C = (A')*B
inputs are:  A, na (rows in A), ma (cols in A)
B, nb (rows in B), mb (cols in B)
and C
*/

void matf_T_mult( float *A, int na, int ma,
                 float *B, int nb, int mb, float *C ) {

	int i, j, k;

	/*QUITIF( na != nb );*/

	matf_init( C, ma, mb, 0.0 );

	for( i = 0; i < ma; i++ )
		for( j = 0; j < mb; j++ )
			for( k = 0; k < na; k++ )
				C[i*mb+j] += A[k*ma+i]*B[k*mb+j];

}


/*  subroutine to add two matrices
C = A + B
inputs are:  A, na (rows in A), ma (cols in A)
B, and C
*/

void matf_add( float *A, int na, int ma, float *B, float *C ) {

	int i, j;

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[i*ma+j] = A[i*ma+j] + B[i*ma+j];

}


/*  subroutine to subtract a matrix from the other
C = A - B
inputs are:  A, na (rows in A), ma (cols in A)
B, and C
*/

void matf_sub( float *A, int na, int ma, float *B, float *C ) {

	int i, j;

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[i*ma+j] = A[i*ma+j] - B[i*ma+j];

}


/*  subroutine to multiply two matrices element by element
C = A.*B
inputs are:  A, na (rows in A), ma (cols in A)
B and C
*/

void matf_elem_mult( float *A[], int na, int ma, float *B[], float *C[] ) {

	int i, j;

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[i][j] = A[i][j]*B[i][j];

}


/*  subroutine to multiply a matrix by a scalar
C = A*b
inputs are:  A, na (rows in A), ma (cols in A)
b (a scalar) and C
*/

void matf_scalar_mult( float *A[], int na, int ma, float b, float *C[] ) {

	int i, j;

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[i][j] = A[i][j]*b;

}


/*  subroutine to transpose a matrix
C = A'
inputs are:  A, na (rows in A), ma (cols in A), C
*/

void matf_transpose( float *A, int na, int ma, float *C ) {

	int i, j;

	matf_init( C, ma, na, 0.0 );

	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			C[j*na+i] = A[i*ma+j];

}


/*  subroutine to compute the trace of a square matrix
ie.  tr(A)
inputs are:  A, na (rows and also columns in A)
*/

float matf_trace( float *A[], int na ) {

	int i;
	float trace_A = 0.0;  /** initialize to zero **/

	for( i = 0; i < na; i++ )
		trace_A += A[i][i];

	return trace_A;

}


/*  subroutine to copy the contents of one matrix to another matrix
B = A
inputs are:  A, na (rows in A), ma (cols in A)
and B
*/

void matf_copy( float *A[], int na, int ma, float *B[] ) {

	int i, j;

	/* this should be a memcpy! */
	for( i = 0; i < na; i++ )
		for( j = 0; j < ma; j++ )
			B[i][j] = A[i][j];

}



static void lu_decompf( float *A, int n, int *indx, float *d ) {

	float vv[MAXSIZE];  /* vv[n] stores implicit scaling of each row */
	register int i, imax = -1, j, k;
	register float big, dum, sum, temp;

	/*if( n > MAXSIZE ) {
	fprintf( stderr, "lu_decompf: n too large\n" );
	return ( -1 );
    }*/

	*(d) = 1.0;

	for( i = 0; i < n; i++ ) {
		big = 0.0;
		for( j = 0; j < n; j++ )
			if( (temp = ABS(A[i*n + j])) > big )
				big = temp;
				/*if ( big == 0.0 ) {
				fprintf( stderr, "lu_decompf: matrix is singular\n" );
				return ( -1 );
	}*/
			vv[i] = 1.0f/big;
	}

	for( j = 0; j < n; j++ ) {
		for( i = 0; i < j; i++ ) {
			sum = A[i*n + j];
			for( k = 0; k < i; k++ )
				sum -= A[i*n + k]*A[k*n + j];
			A[i*n + j] = sum;
		}
		big = 0.0;
		for( i = j; i < n; i++ ) {
			sum = A[i*n + j];
			for( k = 0; k < j; k++ )
				sum -= A[i*n + k]*A[k*n + j];
			A[i*n + j] = sum;
			if( (dum = vv[i]*ABS(sum)) >= big ) {
				big  = dum;
				imax = i;
			}
		}
		if( j != imax ) {
			for( k = 0; k < n; k++ ) {
				dum           = A[imax*n + k];
				A[imax*n + k] = A[j*n + k];
				A[j*n + k]    = dum;
			}
			*(d) = -(*(d));
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if( A[j*n + j] == 0.0 )
			A[j*n + j] = 1.0e-12f;
		if( j != n-1 ) {
			dum = 1.0f/A[j*n + j];
			for ( i = j+1; i < n; i++ ) A[i*n + j] *= dum;
		}
	}

}

/******************************************************************************
*/

static void lu_back_subf( float *A, int n, int *indx,  float *b ) {

	register int    i, ii, ip, j;
	register float sum;

	ii = -1;

	for( i = 0; i < n; i++ ) {
		ip  = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if( ii != -1 )
			for( j = ii; j < i; j++ )
				sum -= A[i*n + j]*b[j];
			else if ( sum )
				ii = i;
			b[i] = sum;
	}

	for( i = n-1; i > -1; i-- ) {
		sum = b[i];
		for ( j = i+1; j < n; j++ )
			sum -= A[i*n + j]*b[j];
		b[i] = sum/A[i*n + i];
	}

}

/******************************************************************************
*/

void matf_invert( float *A, int n, float *Ainv /*, float *det*/ ) {

	int indx[MAXSIZE];   /* index is dimension n */
	float b[MAXSIZE],   /* b is dimension n     */
		Acopy[MAXSIZE*MAXSIZE];   /* Acopy is n by n      */
	register int i, j;

	/* just to remove *det from arguments */
	float determinant, *det;
	det = &determinant;

	/*QUITIF( n > MAXSIZE );*/

	for( i = 0; i < n; i++ ) {
		for( j = 0; j < n; j++ )
			Acopy[i*n + j] = A[i*n + j];
	}

	lu_decompf( Acopy, n, indx, det );

	for( j = 0; j < n; j++ ) {
		*(det) *= Acopy[j*n + j];
		for( i = 0; i < n; i++ )
			b[i] = 0.0;
		b[j] = 1.0;
		lu_back_subf( Acopy, n, indx, b );
		for( i = 0; i < n; i++ )
			Ainv[i*n + j] = b[i];
	}

}

double mat_norm2(double *A, int n)
{
	int i;
	double sum;
	sum = 0;
	for(i=0;i<n;i++)
	{
		sum += A[i]*A[i];
	}
	return sqrt(sum);
}

double mat2d_norm2_sub(double *A, int n, int la)
{
	int i, j;
	double sum;
	sum = 0;
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			sum += A[i*la + j]*A[i*la + j];
		}
	}
	return sqrt(sum);
}

void mat_cross( const double *v1, const double *v2, double *cross ) {

  cross[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);
  cross[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);
  cross[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]);

}

double mat_dotprod( const double *v1, const double *v2, int n)
{
	int i;
	double sum;
	sum = 0;
	for(i=0;i<n;i++)
	{
		sum += v1[i]*v2[i];
	}
	return sum;
}



/* normalize the vector using 2 norm, A and B can be the same vector */
void mat_normalize( const double *A, int n, double *B)
{
	double sum=0;
	double mag=0;
	double eps = 1e-24;
	int i;

	for(i=0; i<n; i++)
		sum += SQ(A[i]);

	if(sum < eps) {printf("\nmat_normalize : very small < %f",eps); sum = eps;}

	mag = sqrt(sum);

	for(i=0; i<n; i++)
		B[i] = A[i]/mag;
}

#define MAT_MAX_QR (64*3)
/* subroutine to calculate the qr decomposion
 * of the transpose of a square matrix. Inputs
 * are the matrix A, the size of the subsection m,
 * and the full size of A, la .
 * output is A = R', upper triangular
 * Uses Householder transformations
 * See Tefethen and Bau Pg. 73 */
void mat_qr_sub_T( double *A, int m, int la ){
    
    
    if( m > MAT_MAX_QR || m > la ){
        printf("mat_qr_T: matrix too big");
        return;
    }
    
    /* triangulate Ut */
    double vtA[MAT_MAX_QR] = {0};
    double vec[MAT_MAX_QR] = {0};
    double d[MAT_MAX_QR]   = {0};
    double sx1;
    
    int j,k,q;
    
    for( j=m-1; j>=0; j-- ) {
        for( k=0; k<=j; k++ ) { 
            vec[k] = A[j*la+k];
        }
        
        d[j] = mat_norm2(vec, j+1);
        sx1 = SIGN(vec[j]);
        vec[j] += sx1*d[j];

        //memset(vtA, 0, j*sizeof( double ) );
        mat_init(vtA, j+1, 1, 0.0 );
        mat_normalize(vec, j+1, vec);
        for( k=0; k<=j; k++ ) {
            for( q=0; q<=j; q++ ) {
                vtA[q] += vec[k]*A[q*la+k];
            }
        }
        
        for( k=0; k<=j; k++ ) {
            for( q=0; q<=j; q++ ) {
                A[q*la+k] -= 2*vec[k]*vtA[q];
            }
        }
    }
    
    return;
    
}


void mat_qr_T( double *A, int m ){
    
    mat_qr_sub_T( A, m, m );
    
    return;
    
}

/* factors a positive definite matrix into
 * input: positive definite matrix A
 *        size n
 * output: upper triangular factor A, where Ain = Aout'*Aout
 * See Tefethen and Bau Pg. 178
 */
int mat_cholesky_UtU( double *A, int n ) {

    int i, j, k;
    double scale;

    //zero sub-diagonal
    for( i=0; i<n; i++ ) {
        for( j=0; j<i; j++ ) {
            A[i*n+j] = 0.0;
        }
    }
    
    for( i=0; i<n; i++ ) {
        for( j=i+1; j<n; j++ ) {
            scale = A[i*n+j]/A[i*n+i];
            for( k=j; k<n; k++ )
                A[j*n+k] -= A[i*n+k]*scale;
        }
        scale = 1/sqrt(A[i*n+i]);
        for( j=i; j<n; j++ )
            A[i*n+j] *= scale;
    }
    
    return 0;

}

/* factors a positive definite matrix into A = LtL
 * input: positive definite matrix A
 *        size n
 * output: lower triangular factor A, where Ain = Aout'*Aout
 * Adapted from Tefethen and Bau Pg. 178
 */
int mat_cholesky_LtL( double *A, int n ) {

    int i, j, k;
    double scale;

    //zero super-diagonal
    for( i=n-2; i>=0; i-- ) {
        for( j=n-1; j>i; j-- ) {
            A[i*n+j] = 0.0;
        }
    }
    
    for( i=n-1; i>=0; i-- ) {
        for( j=i-1; j>=0; j-- ) {
            scale = A[i*n+j]/A[i*n+i];
            for( k=j; k>=0; k-- )
                A[j*n+k] -= A[i*n+k]*scale;
        }
        scale = 1/sqrt(A[i*n+i]);
        for( j=0; j<=i; j++ )
            A[i*n+j] *= scale;
    }
    
    return 0;

}

/* factors a positive definite matrix into U*D*L
 * input: positive definite matrix A
 *        dim n
 *        matrix container size
 * output: upper triangular factor A, 
 *         vector D
 *         where Ain = Aout'*D*Aout
 * Adapted from Tefethen and Bau Pg. 178
 */
int mat_cholesky_LtDL_sub( double *A, double* D, int n, int size ) {

    int i, j, k;
    double scale;

    //zero super-diagonal
    for( i=n-2; i>=0; i-- ) {
        for( j=n-1; j>i; j-- ) {
            A[i*size+j] = 0.0;
        }
    }
    
    for( i=n-1; i>=0; i-- ) {
        for( j=i-1; j>=0; j-- ) {
            scale = A[i*size+j]/A[i*size+i];
            for( k=j; k>=0; k-- )
                A[j*size+k] -= A[i*size+k]*scale;
        }
        D[i] = A[i*size+i];
        scale = 1/D[i];
        for( j=0; j<=i; j++ )
            A[i*size+j] *= scale;
    }
    
    return 0;

}

/* factors a positive definite matrix into U*D*L
 * input: positive definite matrix A
 *        dim n
 * output: upper triangular factor A, 
 *         vector D
 *         where Ain = Aout'*D*Aout
 * Adapted from Tefethen and Bau Pg. 178
 */
int mat_cholesky_LtDL( double *A, double* D, int n ) {
    
    return mat_cholesky_LtDL_sub( A, D, n, n );

}


int mat_eigv_posdefsym2x2( double A[2][2], double eigval[2], double eigvec[2][2] )
{
    static double tol = 1e-6;
    
    if( ABS(A[0][1] - A[1][0]) > tol ) {
        printf( "mat_eigv_posdefsym2x2: matrix not symmetric");
        return 0;
    }
    
    double trace, det, temp;
    trace = A[0][0] + A[1][1];
    det = A[0][0]*A[1][1] - A[0][1]*A[1][0];

    /* eigenvalues */
    temp = sqrt( MAX( (SQ(trace)/4 - det), 0) );
    if( A[0][0] > A[1][1] ){
        eigval[0] = trace/2 + temp;
        eigval[1] = trace/2 - temp;
    } else {
        eigval[0] = trace/2 - temp;
        eigval[1] = trace/2 + temp;
    }

    /* eigenvectors */
    if( A[1][0] >= tol || A[1][0] <= -tol ){
        eigvec[0][0] = eigval[0] - A[1][1];
        eigvec[0][1] = A[1][0];

        eigvec[1][0] = eigval[1] - A[1][1];
        eigvec[1][1] = A[1][0];

    } else {
        eigvec[0][0] = 1;
        eigvec[0][1] = 0;

        eigvec[1][0] = 0;
        eigvec[1][1] = 1;
    }
    
    return 0;
}

