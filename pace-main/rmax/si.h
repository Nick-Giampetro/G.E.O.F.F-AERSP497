#ifndef rmax_si_h
#define rmax_si_h

#include "rmax/si_ref.h"

#if defined(__cplusplus)
extern "C"
{
#endif

void initSI( void );
void updateSI( void );
void updateSIInputs( void );
void gpsDate( struct gpsModel_ref *gps, double time );

void initNovatelModel(struct gpsModel_ref *gps);
void updateNovatelModel(struct gpsModel_ref *gps,double time, double p_b_e_L[3], double v_b_e_L[3], 
                                                 double w_b_e_B[3],double dcm_bl[3][3],double psi);
void updateIMUModel(struct imuModel_ref* imu, double latitude, double w_b_e_B[3],double a_b_e_L[3],double wd_b_e_B[3],double dcm_lb[3][3],double time);

void intersimCheckSumEncode( unsigned char *buf, int byteCount ) ;

#if defined(__cplusplus)
}
#endif

#endif

