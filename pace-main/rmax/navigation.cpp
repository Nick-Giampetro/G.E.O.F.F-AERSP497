#include "esim/util.h"
#include "esim/quat.h"
#include "rmax/onboard_ref.h"
#include "rmax/sensors_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/navigation.h"
#include "rmax/motion_ref.h"  /* to allow use of truth data */

void updateNavigation ( struct onboard_ref* ob )
{
	/* truth data */
	struct vehicle_ref* v = &vehicle;
	struct motionXforms_ref* xforms = v->motion->xforms;
	struct state_ref* state = v->motion->state;
	struct state_ref* std = v->motion->stateDot;	// to get accel

	/* emulated sensor data */
	struct sensors_ref* sen = ob->sensors;
	struct senImu_ref* imu = sen->imu; // imu sensor (gyroscope + accelermeter)
	struct senAGL_ref* agl = sen->agl; // above ground level sensor (sonar)
	struct senGps_ref* gps = sen->gps; // gps sensor
	struct senMagnet_ref* mag = sen->magnet; // magnetometer

	/* navigation filter */
	struct navigation_ref* nav = ob->navigation;
	struct navout_ref* out = nav->out;

	out->itime++;
	out->time = out->itime * nav->set->imuDt;
	for ( int i = 0; i < 3; i++ )
	{
		out->p_b_e_L[i] = state->p_b_e_L[i];
		out->v_b_e_L[i] = state->v_b_e_L[i];
		out->w_b_e_L[i] = xforms->w_b_e_L[i];
		out->v_b_e_B[i] = xforms->v_b_e_B[i];
		out->w_b_e_B[i] = state->w_b_e_B[i];
		out->a_b_e_L[i] = std->v_b_e_L[i];
	}
	out->altitudeAGL = -out->p_b_e_L[2] - vehicleMotion.env->terrainAlt;
	out->rpm = state->omega / ( 2.0 * C_PI ) * 60;
	
	out->batteryPercent = out->batteryPercent - 0.001;

	/* express acceleration in body axis */
	map_vector ( xforms->dcm_lb, out->a_b_e_L, out->a_b_e_B );

	for ( int i = 0; i < 4; i++ )
	{
		out->q[i] = state->e[i];
	}

	for ( int i = 0; i < 3; i++ )
	{
		for ( int j = 0; j < 3; j++ )
		{
			out->dcm_bl[i][j] = xforms->dcm_bl[i][j];
			out->dcm_lb[i][j] = xforms->dcm_lb[i][j];
		}
	}
}