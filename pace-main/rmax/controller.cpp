#include "rmax/onboard_ref.h"
#include "rmax/navigation_ref.h"
#include "rmax/controller_ref.h"
#include "rmax/controller.h"
#include "esim/util.h"

// just for a demonstration
#include <time.h>
#include <random>
unsigned char seed = 0;
double randomController( double a, double b ) {
	if ( 0 == seed )
	{
		seed = 1;
		srand( (unsigned int ) time( 0 ) );
	}
	double random = ( ( double ) rand() ) / ( double ) RAND_MAX;
	double diff = b - a;
	double r = random * diff;
	return a + r;
}

#define MIN_PWM 1000
#define MAX_PWM 2000

void updateControl ( struct onboard_ref* ob )
{
	struct navout_ref* nav = ob->navigation->out; // navigation results
	struct onboardControl_ref* cntrl = ob->control; // controller
	struct actuatorInt_ref* act = ob->actuators; // actuators
	struct obDatalink_ref* data = ob->datalink; // datalink messages

	struct datalinkMessagePWM_ref* pwmout = act->pwmFromUs; // pwm to ESCs

	// a very smart controller, outputting random commands
	act->work->c_delf[0] = randomController( -1, 1 );
	act->work->c_delm[0] = randomController( -1, 1 );
	act->work->c_delm[1] = randomController( -1, 1 );
	act->work->c_delm[2] = randomController( -1, 1 );

	// modes
	if ( -1 == data->up0->button[1] ) 
	{
		data->m0->autopilot = 0; // manual mode
	}
	else if ( 0 == data->up0->button[1] )
	{
		data->m0->autopilot = 0; // manual mode
	}
	else if ( 1 == data->up0->button[1] ) 
	{
		data->m0->autopilot = 1; // auto mode

		// act commands sent to GCS is replaced by controller output
		act->work->delf[0] = act->work->c_delf[0];
		act->work->delm[0] = act->work->c_delm[0];
		act->work->delm[1] = act->work->c_delm[0];
		act->work->delm[2] = act->work->c_delm[2];
	}

	// mixer
	if ( 1 == data->up0->button[0] ) // arm
	{
		data->m0->LaunchState = 1;

		// only throttle and roll
		double thr_pwm  = ( act->work->delf[0] - ( -1 ) ) * ( 2000 - 1000 ) / ( 1 - ( -1 ) ) + 1000; // from [-1,1] to [1000,2000]
		double roll_pwm = ( act->work->delm[0] - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]

		pwmout->channel[0] = ( unsigned short ) LIMIT( thr_pwm - roll_pwm, MIN_PWM, MAX_PWM ); // front-right CW
		pwmout->channel[1] = ( unsigned short ) LIMIT( thr_pwm - roll_pwm, MIN_PWM, MAX_PWM ); // back-right  CCW
		pwmout->channel[2] = ( unsigned short ) LIMIT( thr_pwm + roll_pwm, MIN_PWM, MAX_PWM ); // back-left   CW
		pwmout->channel[3] = ( unsigned short ) LIMIT( thr_pwm + roll_pwm, MIN_PWM, MAX_PWM ); // back-right  CCW

		if ((cntrl->sidewinderCount > 0) && (static_cast<int>(nav->time) % 10 == 0))
			cntrl->sidewinderCount-- ;

	}
	else // disarm
	{
		data->m0->LaunchState = 0;
		for ( unsigned int i = 0; i < 4; i++ )
		{
			pwmout->channel[i] = MIN_PWM;
		}
	}
}