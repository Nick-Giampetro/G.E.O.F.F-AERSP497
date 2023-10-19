#ifndef util_h
#define util_h

#define C_PI 3.14159265358979323846264338327950288419716939937511
#define C_E  2.71828182845904523536028747135266249775724709369995
#define C_SOL 299792458.0 //speed of light in m/s
#define C_DEG2RAD  0.017453292519943295
#define C_RAD2DEG  57.295779513082323
#define C_RPM2RPS  0.10471975511966
#define C_RPS2RPM  9.54929658551372

#define C_FPS2KPH 1.09728
#define C_KPH2FPS 0.911344415281423
#define C_FPS2KT 0.592483801296
#define C_KT2FPS 1.6878098571
#define C_FPS2MPH 0.681818181818182
#define C_MPH2FPS 1.466666666666667
#define C_FT2NM 0.000164578833693
#define C_NM2FT 6076.11548556
#define C_FT2M  0.3048
#define C_FT2KM 0.0003048
#define C_KM2FT 3280.83989501312
#define C_M2FT 3.280839895
#define C_M2CM 100
#define C_CM2M 0.01
#define C_M2MM 1000
#define C_MM2M 0.001
#define C_FT2SM (1/5280)
#define C_SM2FT 5280
#define C_USGALLON2OUNCES 128
#define C_PA2PSI 0.000145037738
#define C_PSI2PA 6894.75729 
#define C_BAR2PA 100000

#define C_KG2SLUG 0.0685217659
#define C_SLUG2KG 14.5939029
#define C_KGM2_TO_SLUGFT2   0.7375615694452221

#define C_KT2KPH (C_KT2FPS*C_FPS2KPH)
#define C_KT2MPH (C_KT2FPS*C_FPS2MPH)
#define C_KT2MPS (C_KT2FPS*C_FT2M)

#define CF_PI 3.14159265359f
#define CF_2PI 6.28318530718f
#define CF_E  2.71828182846f
#define CF_DEG2RAD 0.017453292519943295f
#define CF_RAD2DEG 57.295779513082323f

#define CF_FPS2KT 0.592483801296f
#define CF_KT2FPS 1.6878098571f
#define CF_FT2NM 0.000164578833693f
#define CF_NM2FT 6076.11548556f
#define CF_FT2M 0.3048f
#define CF_M2FT 3.280839895f

#ifndef MIN
#define MIN(x1,x2) ((x1)<(x2)?(x1):(x2))
#endif
#ifndef MAX
#define MAX(x1,x2) ((x1)>(x2)?(x1):(x2))
#endif
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))
#define ABS(x) ((x)<0.0?(-(x)):(x))
#define SQ(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))
#define SIGN(x) ((x)<(0)?(-1):((x)>(0)?(1):(0)))
#define INRANGE(x,x1,x2) ((x)<=(x2)&&(x)>=(x1)?1:0)

// treats given pointer to array as an n-dimensional array and gets the (i,j,k,l) element
// given macros are for 1,2,3,4 arrays
#define GET1D( MAT, N0, I )                         *( (double*)(MAT) + (I) )
#define GET2D( MAT, N0, N1, I, J )                  *( (double*)(MAT) + (I)*(N1)           + (J) )
#define GET3D( MAT, N0, N1, N2, I, J, K )           *( (double*)(MAT) + (I)*(N1)*(N2)      + (J)*(N2)      + (K) )
#define GET4D( MAT, N0, N1, N2, N3, I, J, K, L )    *( (double*)(MAT) + (I)*(N1)*(N2)*(N3) + (J)*(N2)*(N3) + (K)*(N3) + (L) )

// same macro just different name
#define SET1D( MAT, N0, I )                         *( (double*)(MAT) + (I) )
#define SET2D( MAT, N0, N1, I, J )                  *( (double*)(MAT) + (I)*(N1)           + (J) )
#define SET3D( MAT, N0, N1, N2, I, J, K )           *( (double*)(MAT) + (I)*(N1)*(N2)      + (J)*(N2)      + (K) )
#define SET4D( MAT, N0, N1, N2, N3, I, J, K, L )    *( (double*)(MAT) + (I)*(N1)*(N2)*(N3) + (J)*(N2)*(N3) + (K)*(N3) + (L) )

#define FILE_READ_INT(FILE, VAR, NMEMB)    fread( (VAR), 4,    (NMEMB), (FILE) )
#define FILE_READ_DOUBLE(FILE, VAR, NMEMB) fread( (VAR), 8,    (NMEMB), (FILE) )

// cross product
#define VCROSS0(A0,A1,A2,B0,B1,B2) ((A1)*(B2) - (A2)*(B1))
#define VCROSS1(A0,A1,A2,B0,B1,B2) ((A2)*(B0) - (A0)*(B2))
#define VCROSS2(A0,A1,A2,B0,B1,B2) ((A0)*(B1) - (A1)*(B0))

#if defined(__cplusplus)
extern "C"
{
#endif

double hmodDeg( double h );
double hmod360( double h );
double hmodRad( double h );
float  hmodRadf( float h );

/** bisection location */
int locateBisect(double *xx, int n, double x);

// table lookup routines
double lookup1d(double *yy, double *xx, int n, double x);
double lookup2d(double *yy,
				double *xx0, double *xx1,
				int    n0,   int    n1,
				double x0,   double x1) ;

// Allows for a larger stored table, even though the breakpoints and actual
//  data are smaller.
double lookup2dFlexSize(double *yy,
						double *xx0, double *xx1,
						int    n0,   int    n1,
						double x0,   double x1,
						int n0Stored, int n1Stored) ;


double lookup3d(double *yy,
				double *xx0, double *xx1, double *xx2,
				int    n0,   int    n1,   int n2,
				double x0,   double x1,   double x2);

double lookup4d(double *yy,
				double *xx0, double *xx1, double *xx2, double *xx3,
				int    n0,   int    n1,   int    n2,   int n3,
				double x0,   double x1,   double x2,   double x3);

// vector version of lookup1d (useful for resampling data)
// given a table xx, yy; samples points at x (of length nx)
// and stores it in y
void lookup1dv(double *yy, double *xx, int n, double *y, double *x, int nx);



unsigned short endian_swap_ushort(unsigned short x);
unsigned int endian_swap_uint(unsigned int x);

// calculates the Hamming distance between two vectors of lenght given by bytes
unsigned int calcHamming( const char vec1[], const char vec2[], int bytes );

#if defined(CUSTOM_ROUND_FUNCTION)
    double round(double);
#endif

#if defined(__cplusplus)
}
#endif


#if defined(__cplusplus)



#endif // c++

#endif



