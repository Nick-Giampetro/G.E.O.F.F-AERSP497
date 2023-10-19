#ifndef rmax_matrix_h
#define rmax_matrix_h

#if defined(__cplusplus)
extern "C"
{
#endif

#define MAXSIZE 25
#define MAXFLEXSIZE 350
typedef void *POINTER;

/* extra stuff */
void lowpass( int init, double input, double dt, double tc, double *state );
double notch( int init, double input, double dt,
              double frequency, double spread, double attenuation, double *state );
void time_delay_in( int init, double in, double *buffer, int bufferSize );
double time_delay_out( double iDelay, double *buffer, int bufferSize );
double time_delay( int init, double in, double iDelay, double *buffer, int bufferSize );
double latency( int init, double in, double *buffer, int *index, int size );

/* prototypes of matrix manipulation routines */
void mat_init( double *in, int rows, int cols, double init_val );
void mat_init_sub( double *in, int rows, int cols, int size, double init_val );
void mat_diag_init( double *in, int rows, int cols, double init_val );

void mat_mult( double *A, int na, int ma,
               double *B, int nb, int mb,
               double *C );
void mat_mult_sub( double *A, int na, int ma, int la,
                   double *B, int nb, int mb, int lb,
                   double *C, int lc);
void mat_T_mult( double *A, int na, int ma,
                 double *B, int nb, int mb,
                 double *C );
void mat_T_mult_sub( double *A, int na, int ma, int la,
                     double *B, int nb, int mb, int lb, 
                     double *C,                 int lc );
void mat_mult_T( double *A, int na, int ma,
                 double *B, int nb, int mb,
                 double *C );
void mat_mult_T_sub( double *A, int na, int ma, int la,
                 double *B, int nb, int mb, int lb,
                 double *C, int lc );
void mat_elem_mult( double *A, int na, int ma, double *B,
                    double *C );
void mat_scalar_mult( double *A, int na, int ma, double b, double *C );
void mat_add( double *A, int na, int ma, double *B,
              double *C );
void mat_sub( double *A, int na, int ma, double *B,
              double *C );
void mat_transpose( double *A, int na, int ma,
                    double *C );
double mat_trace( double *A, int na );
void mat_copy( double *A, int na, int ma,
               double *B );
void mat_invert( double *A, int n, double *Ainv );

/** For flexible-sized matrices up to a max allocated size */
void mat_invert_flexsize( double *A, int n, int maxn, double *Ainv );

/** Allows a flexible-sized matrix with maxn being the fully allocated matrix size */
void matf_invert_flexsize( float *A, int n, int maxn, float *Ainv );

double mat_norm2(double *A, int n);

double mat2d_norm2_sub(double *A, int n, int la);

void mat_cross( const double *v1, const double *v2, double *cross );

double mat_dotprod( const double *v1, const double *v2, int n);

/* normalize the vector using 2 norm */
void mat_normalize( const double *A, int n, double *B);

/* floating point versions */
void matf_init( float *in, int rows, int cols, float init_val );
void matf_mult( float *A, int na, int ma,
               float *B, int nb, int mb,
               float *C );
void matf_T_mult( float *A, int na, int ma,
                 float *B, int nb, int mb,
                 float *C );
void matf_mult_T( float *A, int na, int ma,
                 float *B, int nb, int mb,
                 float *C );
void matf_elem_mult( float *A[], int na, int ma, float *B[],
                    float *C[] );
void matf_scalar_mult( float *A[], int na, int ma, float b,
                      float *C[] );
void matf_add( float *A, int na, int ma, float *B,
              float *C );
void matf_sub( float *A, int na, int ma, float *B,
              float *C );
void matf_transpose( float *A, int na, int ma,
                    float *C );
float matf_trace( float *A[], int na );
void matf_copy( float *A[], int na, int ma,
               float *B[] );
void matf_invert( float *A, int n, float *Ainv );

/* some other useful algorithms */
void mat_qr_sub_T( double *A, int m, int n );
void mat_qr_T( double *A, int m );

int mat_cholesky_LtL( double *A, int n );
int mat_cholesky_UtU( double *A, int n );
int mat_cholesky_LtDL( double *A, double* D, int n );
int mat_cholesky_LtDL_sub( double *A, double* D, int n, int size );

/* Computes the eigenvalues and eigenvectors for a 2x2 symmetric positive
 * definite matrix
 * inputs
 *      A: 2x2 symmetric positive definite matrix A
 *      eigval: 2-array for eigenvalue output
 *      eigvec: 2x2 array for eigenvector output
 */
int mat_eigv_posdefsym2x2( double A[2][2], double eigval[2], double eigvec[2][2] );

#if defined(__cplusplus)
}
#endif



#endif

