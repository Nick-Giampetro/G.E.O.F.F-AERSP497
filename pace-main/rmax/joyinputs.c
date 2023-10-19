// TODO_JPD: Add copywrite notice


// Function inputs
#include "rmax/sensors.h"

// Database inputs
#include "esim/input_ref.h"
#include "rmax/joyinputs_ref.h"
#include "rmax/sensors_ref.h" // Only for RMAX controls, should probably be cleaned up
#include "rmax/si_ref.h"      // Only for RMAX controls, should probably be cleaned up

#include "esim/util.h"
#include "rmax/joyinputs.h"

float doStickBlend( float first, float second ) {

	if ( first == 0 )             return second;
	if ( second == 0 )             return first;
	if ( first < 0 && second > 0 ) return 0;
	if ( first > 0 && second < 0 ) return 0;
	if ( first > 0 && second > 0 ) return MAX( first, second );
	return MIN( first, second );

}


void updateJoyInputs( struct controlInput_ref* ci, double secTime)
{
	struct input_ref         *joy    = &esimInput;
    int iJoy;
    
    iJoy = ci->whichJoyInput;
    if( iJoy > 15 ){
        ci->whichJoyInput = iJoy = 15;
    } else if( iJoy < 0 ){
        ci->whichJoyInput = iJoy = 0;
    }

	/* PILOT MODE DIRECT CONTROL HELI OR AIRPLANE */
	switch( ci->mode ) {

	case MOTIONINPUT_JOYSTICK:
		switch( ci->joystickMode ) {
		default:
		case JOYSTICK_MODE_2:
			ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			ci->rudder->output  = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			ci->throttle->output = -joy->joyInput[iJoy]->joyAxis[2];
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];
			break;

		case JOYSTICK_MODE_1:
			ci->pitch->output   = -MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			ci->roll->output    = -MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			ci->rudder->output  = -MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			ci->throttle->output = joy->joyInput[iJoy]->joyAxis[2];
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];

			break;

		case JOYSTICK_MODE_3:
			ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			ci->rudder->output  = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			ci->throttle->output = -joy->joyInput[iJoy]->joyAxis[2];
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];
			break;

		case JOYSTICK_MODE_3_REV:
			ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			ci->rudder->output  = -MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			ci->throttle->output = -joy->joyInput[iJoy]->joyAxis[2];
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];
			break;

		case JOYSTICK_MODE_2_AIRPLANE:
			ci->rudder->output  = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			ci->throttle->output = -joy->joyInput[iJoy]->joyAxis[2];
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];
			break;

		case JOYSTICK_MODE_FIGHTERSTICK:
			ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			if (joy->joyInput[iJoy]->button[15]) {
				ci->rudder->output = -1.0;
			} else if (joy->joyInput[iJoy]->button[13]) {
				ci->rudder->output =  1.0;
			} else {
				ci->rudder->output =  0.0;
			}

			if (joy->joyInput[iJoy]->button[12]) {
				ci->throttle->output = 1.0;
			} else if (joy->joyInput[iJoy]->button[14]) {
				ci->throttle->output = -1.0;
			} else {
				ci->throttle->output = 0.0;
			}
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];

			break;

		case JOYSTICK_MODE_LOGITECHRUMBLE2:
/*
                    Logitech RumblePad 2 Button Map

               7(bottom)           ||                  8(bottom)
           5(top)                  ||                      6(top)
      /-------------\              ||               /-------------\
     /               \             ||              /               \
    /      H0         \---------------------------/        /---\    \
   /       /\                                              | 4 |     \
  /        ||              < 9 >        < 10 >             \---/      \
 |   H3 /--  --\ H1                                   /---\     /---\  |
 |      \--  --/            < M >      <  V >         | 1 |     | 3 |  |
 |         ||                                         \---/     \---/  |
 |         \/                                              /---\       |
 |         H2            11    ----------     12           | 2 |       |
 |               /----\       /           \       /----\   \---/       |
  \             /      \-----/             \-----/      \             /
   \           /                                         \           /
    \---------/                                           \---------/

*/
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[2] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[2] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->rudder->output  = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->throttle->output = -( MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->throttle->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->throttle->deadBand ) );
			ci->air->output = joy->joyInput[iJoy]->button[5];
			ci->ground->output = (joy->joyInput[iJoy]->button[7] && joy->joyInput[iJoy]->button[0]);
			ci->shutdown->output = (joy->joyInput[iJoy]->button[7] || joy->joyInput[iJoy]->button[0]);
			ci->arm->output = joy->joyInput[iJoy]->button[7];
			ci->suppressStick->output = joy->joyInput[iJoy]->button[7];
			ci->safeOn->output = joy->joyInput[iJoy]->button[0];
			ci->safeOff->output = joy->joyInput[iJoy]->button[2];
			//ci->joySiManD->output = joy->joyInput[iJoy]->button[1];

			if( joy->joyInput[iJoy]->button[6] == 0 )
				ci->dash->output = joy->joyInput[iJoy]->button[4];

			if( joy->joyInput[iJoy]->button[8] && joy->joyInput[iJoy]->button[6] && ci->old46 == 0 )
				ci->manOverride->output = !ci->manOverride->output;
			ci->old46 = joy->joyInput[iJoy]->button[8] & joy->joyInput[iJoy]->button[6];

			ci->loadRunPlan->output = joy->joyInput[iJoy]->button[3];
			ci->planToggle->output  = joy->joyInput[iJoy]->button[9];
			ci->gpsDenied->output = (joy->joyInput[iJoy]->button[6] && joy->joyInput[iJoy]->button[11]);
			ci->gpsDenRestGps->output = (joy->joyInput[iJoy]->button[6] && !joy->joyInput[iJoy]->button[11]);
			ci->gpsDenRestPos->output = (joy->joyInput[iJoy]->button[11] && !joy->joyInput[iJoy]->button[6]);
			ci->suppressSonar->output = (joy->joyInput[iJoy]->button[11]);

			if( joy->joyInput[iJoy]->button[10]==1 && ci->videoTogglePrev==0 ) {
				ci->videoToggle->output = -1; /* swap video source */
			}
			ci->videoTogglePrev = joy->joyInput[iJoy]->button[10];

			ci->zoomIn->output = joy->joyInput[iJoy]->hatswitch[0];
			ci->zoomOut->output = joy->joyInput[iJoy]->hatswitch[2];

			break;

		case JOYSTICK_MODE_SAITEKFPS:
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->rudder->output  = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[2] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[2] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->throttle->output = -( MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->throttle->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->throttle->deadBand ) );
			ci->air->output = joy->joyInput[iJoy]->button[5];
			ci->ground->output = (joy->joyInput[iJoy]->button[7] && joy->joyInput[iJoy]->button[0]);
			ci->shutdown->output = (joy->joyInput[iJoy]->button[7] || joy->joyInput[iJoy]->button[0]);
			ci->arm->output = joy->joyInput[iJoy]->button[7];
			ci->suppressStick->output = joy->joyInput[iJoy]->button[7];
			ci->safeOn->output = joy->joyInput[iJoy]->button[0];
			ci->safeOff->output = joy->joyInput[iJoy]->button[2];
			if( joy->joyInput[iJoy]->button[6] == 0 )
				ci->dash->output = joy->joyInput[iJoy]->button[4];

			if( joy->joyInput[iJoy]->button[8] && joy->joyInput[iJoy]->button[6] && ci->old46 == 0 )
				ci->manOverride->output = !ci->manOverride->output;
			ci->old46 = joy->joyInput[iJoy]->button[8] & joy->joyInput[iJoy]->button[6];

			ci->loadRunPlan->output = joy->joyInput[iJoy]->button[3];
			ci->planToggle->output  = joy->joyInput[iJoy]->button[9];
			ci->gpsDenied->output = (joy->joyInput[iJoy]->button[6] && joy->joyInput[iJoy]->button[11]);
			ci->gpsDenRestGps->output = (joy->joyInput[iJoy]->button[6] && !joy->joyInput[iJoy]->button[11]);
			ci->gpsDenRestPos->output = (joy->joyInput[iJoy]->button[11] && !joy->joyInput[iJoy]->button[6]);
			ci->suppressSonar->output = (joy->joyInput[iJoy]->button[11]);
			break;

		case JOYSTICK_MODE_XBOX360:
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[4] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[4] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->rudder->output  = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->throttle->output = -( MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->throttle->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->throttle->deadBand ) );
			ci->safeOff->output = joy->joyInput[iJoy]->button[1];
			ci->safeOn->output = joy->joyInput[iJoy]->button[2];
			ci->air->output = joy->joyInput[iJoy]->button[5];
			ci->ground->output = (joy->joyInput[iJoy]->button[2] && (joy->joyInput[iJoy]->joyAxis[2] < -0.9));
			ci->shutdown->output = (joy->joyInput[iJoy]->button[2] || joy->joyInput[iJoy]->button[5]);
			ci->arm->output = (joy->joyInput[iJoy]->joyAxis[2] < -0.9);
			ci->suppressStick->output = (joy->joyInput[iJoy]->joyAxis[2] < -0.9);
			//ci->joySiMan->output = joy->joyInput[iJoy]->button[6];
			ci->loadRunPlan->output = joy->joyInput[iJoy]->button[3];
			ci->planToggle->output  = joy->joyInput[iJoy]->button[7];
			ci->gpsDenied->output = (joy->joyInput[iJoy]->button[0] && joy->joyInput[iJoy]->button[4]);
			if (joy->joyInput[iJoy]->joyAxis[2] > -0.1 && joy->joyInput[iJoy]->joyAxis[2] < 0.9)
				ci->dash->output = joy->joyInput[iJoy]->button[4];
			break;

		case JOYSTICK_MODE_ESTERLINE:
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->roll->output    = -MAX( ABS( joy->joyInput[iJoy]->joyAxis[4] ) - ci->roll->deadBand, 0.0 )*SIGN( joy->joyInput[iJoy]->joyAxis[4] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->rudder->output =  MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->pitch->output    =    MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->pitch->deadBand, 0.0 )*SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->throttle->output = -( MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->throttle->deadBand, 0.0 )*SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->throttle->deadBand ) );

			ci->air->output    = joy->joyInput[iJoy]->button[13];   // air mode
			ci->ground->output = (joy->joyInput[iJoy]->button[15] && joy->joyInput[iJoy]->button[14]);   // ground mode
			ci->shutdown->output = (joy->joyInput[iJoy]->button[15] || joy->joyInput[iJoy]->button[14]);
			ci->arm->output = joy->joyInput[iJoy]->button[15];
			ci->suppressStick->output = joy->joyInput[iJoy]->button[15];
			ci->safeOn->output   = joy->joyInput[iJoy]->button[14];   // stop rotor
			ci->safeOff->output  = joy->joyInput[iJoy]->button[6];    // start rotor
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];    // ??

			// enable dash/turbo mode if the dash button is pressed but only if manual control mode is NOT pressed
			if( joy->joyInput[iJoy]->button[21] == 0 ) {
				ci->dash->output = joy->joyInput[iJoy]->button[12];
			}

			// check the buttons for manual control mode
			// toggle mode if both buttons are pressed AND the previous state of the buttons was not pressed
			if( joy->joyInput[iJoy]->button[21] && joy->joyInput[iJoy]->button[20] && ci->old46 == 0 )
				ci->manOverride->output = !ci->manOverride->output;
			ci->old46 = joy->joyInput[iJoy]->button[21] & joy->joyInput[iJoy]->button[20];

			break;

		case JOYSTICK_MODE_2_TWINSTAR:
			ci->rudder->output  = -MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			ci->throttle->output = -joy->joyInput[iJoy]->joyAxis[2];
			ci->joySiMan->output = joy->joyInput[iJoy]->button[1];
			break;

		case JOYSTICK_MODE_LOGITECH_F310:
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->roll->output    = MAX( ABS( joy->joyInput[iJoy]->joyAxis[4] ) - ci->roll->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[4] )/MAX( 0.01, 1.0 - ci->roll->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->rudder->output  = MAX( ABS( joy->joyInput[iJoy]->joyAxis[0] ) - ci->rudder->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[0] )/MAX( 0.01, 1.0 - ci->rudder->deadBand );
			if( ci->rightVirtualJoystickInUse == 0 )
				ci->pitch->output   = MAX( ABS( joy->joyInput[iJoy]->joyAxis[3] ) - ci->pitch->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[3] )/MAX( 0.01, 1.0 - ci->pitch->deadBand );
			if( ci->leftVirtualJoystickInUse == 0 )
				ci->throttle->output = -( MAX( ABS( joy->joyInput[iJoy]->joyAxis[1] ) - ci->throttle->deadBand, 0.0 )*
				SIGN( joy->joyInput[iJoy]->joyAxis[1] )/MAX( 0.01, 1.0 - ci->throttle->deadBand ) );
			ci->air->output = joy->joyInput[iJoy]->button[5];
			ci->ground->output = ( joy->joyInput[iJoy]->button[2] && ( joy->joyInput[iJoy]->joyAxis[2] < -0.5 ) );
			ci->shutdown->output =  (joy->joyInput[iJoy]->button[2] || ( joy->joyInput[iJoy]->joyAxis[2] < -0.5 ) );
			ci->arm->output = joy->joyInput[iJoy]->button[2];
			ci->suppressStick->output = joy->joyInput[iJoy]->button[2];
			ci->safeOn->output = joy->joyInput[iJoy]->button[0];
			ci->safeOff->output = joy->joyInput[iJoy]->button[1];

			if( joy->joyInput[iJoy]->joyAxis[2] < 0.5 )
				ci->dash->output = joy->joyInput[iJoy]->button[4];

			if( joy->joyInput[iJoy]->joyAxis[2] > 0.5 && joy->joyInput[iJoy]->button[6] && ci->old46 == 0 )
				ci->manOverride->output = !ci->manOverride->output;
			ci->old46 = ( joy->joyInput[iJoy]->joyAxis[2] > 0.5 ) & joy->joyInput[iJoy]->button[6];

			ci->loadRunPlan->output = joy->joyInput[iJoy]->button[3];
			ci->planToggle->output  = joy->joyInput[iJoy]->button[7];
			ci->gpsDenied->output = (( joy->joyInput[iJoy]->joyAxis[2] > 0.5 ) && joy->joyInput[iJoy]->button[9]);
			ci->gpsDenRestGps->output = (( joy->joyInput[iJoy]->joyAxis[2] > 0.5 ) && !joy->joyInput[iJoy]->button[9]);
			ci->gpsDenRestPos->output = (joy->joyInput[iJoy]->button[9] && !( joy->joyInput[iJoy]->joyAxis[2] > 0.5 ));
			ci->suppressSonar->output = (joy->joyInput[iJoy]->button[9]);

			if( joy->joyInput[iJoy]->button[8]==1 && ci->videoTogglePrev==0 ) {
				ci->videoToggle->output = -1; /* swap video source */
			}
			ci->videoTogglePrev = joy->joyInput[iJoy]->button[8];

			ci->zoomIn->output = joy->joyInput[iJoy]->hatswitch[0];
			ci->zoomOut->output = joy->joyInput[iJoy]->hatswitch[2];

			break;

		case JOYSTICK_MODE_FLYSKY:
		if ( ci->mode == MOTIONINPUT_JOYSTICK )
		{
			ci->roll->output = LIMIT ( MAX ( ABS ( joy->joyInput[iJoy]->joyAxis[0] ) - ci->roll->deadBand, 0.0 ) *
																 SIGN ( joy->joyInput[iJoy]->joyAxis[0] ) / MAX ( 0.01, 0.7 - ci->roll->deadBand ), ci->roll->minValue, ci->roll->maxValue );
			ci->pitch->output = LIMIT ( MAX ( ABS ( joy->joyInput[iJoy]->joyAxis[1] ) - ci->pitch->deadBand, 0.0 ) *
																	SIGN ( joy->joyInput[iJoy]->joyAxis[1] ) / MAX ( 0.01, 0.7 - ci->pitch->deadBand ), ci->pitch->minValue, ci->pitch->maxValue );
			ci->throttle->output = LIMIT ( joy->joyInput[iJoy]->joyAxis[2] / 0.7, ci->throttle->minValue, ci->throttle->maxValue );
			ci->rudder->output = LIMIT ( MAX ( ABS ( joy->joyInput[iJoy]->joyAxis[5] ) - ci->rudder->deadBand, 0.0 ) *
																	 SIGN ( joy->joyInput[iJoy]->joyAxis[5] ) / MAX ( 0.01, 0.7 - ci->rudder->deadBand ), ci->rudder->minValue, ci->rudder->maxValue );
			ci->arm->output = ( short ) (joy->joyInput[iJoy]->joyAxis[3] / 0.7);
			ci->manOverride->output = ( short ) (joy->joyInput[iJoy]->joyAxis[4] / 0.7);
		}
		break;

		case 99:
			{
				// Mappable

				// Only mappable inputs support multiple joysticks

				// Go through each of the analog and digital axes - unfortunately, since the directories have to be defined individually
				//  instead of in an array, a loop cannot be used...
				// Analog functions - only update if the virtual joysticks are not in use
				if( ( ci->rightVirtualJoystickInUse == 0 ) &&
					( ci->leftVirtualJoystickInUse == 0 )  )
				{
					updateAnalogFunction(ci->roll, joy, ci);
					updateAnalogFunction(ci->pitch, joy, ci);
					updateAnalogFunction(ci->rudder, joy, ci);
					updateAnalogFunction(ci->throttle, joy, ci);
					updateAnalogFunction(ci->pan, joy, ci);
					updateAnalogFunction(ci->tilt, joy, ci);
					updateAnalogFunction(ci->zoom, joy, ci);
				}

				// Digital functions
				updateDigitalFunction(ci->manOverride, joy, secTime, ci);
				updateDigitalFunction(ci->arm, joy, secTime, ci);
				updateDigitalFunction(ci->ground, joy, secTime, ci);
				updateDigitalFunction(ci->groundLanding, joy, secTime, ci);
				updateDigitalFunction(ci->air, joy, secTime, ci);
				updateDigitalFunction(ci->airTakeoff, joy, secTime, ci);
				updateDigitalFunction(ci->safeOff, joy, secTime, ci);
				updateDigitalFunction(ci->safeOn, joy, secTime, ci);
				updateDigitalFunction(ci->shutdown, joy, secTime, ci);
				updateDigitalFunction(ci->suppressStick, joy, secTime, ci);
				updateDigitalFunction(ci->dash, joy, secTime, ci);
				updateDigitalFunction(ci->gpsDenied, joy, secTime, ci);
				updateDigitalFunction(ci->gpsDenRestGps, joy, secTime, ci);
				updateDigitalFunction(ci->gpsDenRestPos, joy, secTime, ci);
				updateDigitalFunction(ci->suppressSonar, joy, secTime, ci);
				updateDigitalFunction(ci->zoomIn, joy, secTime, ci);
				updateDigitalFunction(ci->zoomOut, joy, secTime, ci);
				updateDigitalFunction(ci->loadRunPlan, joy, secTime, ci);
				updateDigitalFunction(ci->planToggle, joy, secTime, ci);
				updateDigitalFunction(ci->videoToggle, joy, secTime, ci);
				updateDigitalFunction(ci->joySiMan, joy, secTime, ci);
				updateDigitalFunction(ci->rtb, joy, secTime, ci);
				updateDigitalFunction(ci->stopPlan, joy, secTime, ci);
				updateDigitalFunction(ci->systemSafety, joy, secTime, ci);
				// Camera functions
				updateDigitalFunction(ci->camStop, joy, secTime, ci);
				updateDigitalFunction(ci->camPause, joy, secTime, ci);
				updateDigitalFunction(ci->camPlay, joy, secTime, ci);
				updateDigitalFunction(ci->camRewind, joy, secTime, ci);
				updateDigitalFunction(ci->camFF, joy, secTime, ci);
				updateDigitalFunction(ci->camRecord, joy, secTime, ci);
				updateDigitalFunction(ci->camPowerOn, joy, secTime, ci);
				updateDigitalFunction(ci->tglCamFocusManAuto, joy, secTime, ci);
				updateDigitalFunction(ci->camManFocusFarther, joy, secTime, ci);
				updateDigitalFunction(ci->camManFocusCloser, joy, secTime, ci);
				updateDigitalFunction(ci->camModeJS, joy, secTime, ci);
				updateDigitalFunction(ci->camModeStow, joy, secTime, ci);
				updateDigitalFunction(ci->camModeGeo, joy, secTime, ci);
				updateDigitalFunction(ci->camModeVideo, joy, secTime, ci);
				updateDigitalFunction(ci->camModeToggle, joy, secTime, ci);
				updateDigitalFunction(ci->videoOverlay, joy, secTime, ci);

				updateDigitalFunction(ci->yellowButton, joy, secTime, ci);
				updateDigitalFunction(ci->whiteButton, joy, secTime, ci);
				updateDigitalFunction(ci->blueButton, joy, secTime, ci);
				updateDigitalFunction(ci->greenButton, joy, secTime, ci);
				updateDigitalFunction(ci->bigRedButton, joy, secTime, ci);

				break;
			}
		}
		break;

	case MOTIONINPUT_MOUSE:
	default:
		/* do nothing, handled in scene.c */
		break;
	}
}

void updateDigitalFunction( struct digitalFunction_ref* digiFunc, struct input_ref* joy, double secTime, struct controlInput_ref* mc  )
{
	// Only update whether it is active if in use
	if( digiFunc->inUse )
	{
		// First step, determine whether it is currently (the correct combination is pressed)
		unsigned char rawInput = 1; // Trying to prove it is not active
		int index;
		// On button combination
		if( digiFunc->numOnButtons > 0 )
		{
			if( digiFunc->onButtonsAnd )
			{
				for( index = 0; ((index < digiFunc->numOnButtons) && rawInput); ++index )
				{
					rawInput &= (joy->joyInput[digiFunc->joystickId]->button[digiFunc->onButton[index]] == 1);
				}
			}
			else
			{
				// Default to not active and prove active
				rawInput = 0;
				for( index = 0; ((index < digiFunc->numOnButtons) && !rawInput); ++index )
				{
					rawInput |= (joy->joyInput[digiFunc->joystickId]->button[digiFunc->onButton[index]] == 1);
				}
			}

			// This logic is only necessary if there are On buttons
			if( rawInput )
			{
				if( digiFunc->offBtnLockout )
				{
					// Do not activate if the Off Button Lockout is enabled
					rawInput = 0;
				}
			}
			else if( digiFunc->offBtnLockout)
			{
				// Turn off the lockout
				digiFunc->offBtnLockout = 0;
			}
			// else: nothing to do
		}

		// Go on only if still could be active
		// Off button combination
		if( rawInput && (digiFunc->numOffButtons > 0))
		{
			for( index = 0; ((index < digiFunc->numOffButtons) && rawInput); ++index )
			{
				rawInput &= (joy->joyInput[digiFunc->joystickId]->button[digiFunc->offButton[index]] == 0);
			}

			if( !rawInput )
			{
				// All the appropriate buttons are pressed, but a NOT button is pressed as well.  The danger here
				//  is that the NOT button could be released and the function would activate.  Since NOT buttons
				//  are used to distinguish two overlapping functions, there is a release sequence defined, which
				//  is a problem.  Therefore, if all appropriate buttons are pressed and a NOT button is pressed,
				//  the appropriate buttons must be release (at least 1 of them) before they can be pressed together
				//  again to activate the function.
				digiFunc->offBtnLockout = 1;
			}
		}
		// Go on only if still could be active
		// Hat switch combination
		if( rawInput && (digiFunc->numHatSwitch > 0 ))
		{
			for( index = 0; ((index < digiFunc->numHatSwitch) && rawInput); ++index )
			{
				rawInput &= (joy->joyInput[digiFunc->joystickId]->hatswitch[digiFunc->hatSwitch[index]] == 1);
			}
		}
		// Go on only if still could be active
		// Analog axis
		if( rawInput && (digiFunc->analogAxis >= 0 ))
		{
			float reverseMult = ((digiFunc->analogPositive == 1) ? 1.0F : -1.0F);
			rawInput &= ((joy->joyInput[digiFunc->joystickId]->joyAxis[digiFunc->analogAxis] * reverseMult) > ABS(digiFunc->analogMin));
		}

		// Now that we know if the sequence is active, do something with it
		if( rawInput )
		{
			if( !digiFunc->releaseToReactivate || (digiFunc->releaseToReactivate && digiFunc->wasReleased) )
			{
				// Okay, it is truly active.  Next check whether there is a activation period
				if( digiFunc->secHold <= 0.0 )
				{
					// No activation period - activate the function
					digiFunc->input = 1;
					digiFunc->wasReleased = 0;
					digiFunc->secRemaining = 0.0;
				}
				else
				{
					if( digiFunc->secTimePressed <= 0.0 )
					{
						// Set the start timer
						digiFunc->secTimePressed = secTime;
						digiFunc->secRemaining = digiFunc->secHold;
					}
					else if( secTime > (digiFunc->secTimePressed + digiFunc->secHold)) // Check whether the time has elapsed to activate the function
					{
						digiFunc->input = 1;
						digiFunc->wasReleased = 0;
						digiFunc->secRemaining = 0.0;
					}
					else
					{
						// Update the seconds remaining
						digiFunc->secRemaining = digiFunc->secHold - (secTime - digiFunc->secTimePressed);
					}
				}
			}
			else if (digiFunc->releaseToReactivate && !digiFunc->wasReleased)
			{
				// Only activate on change
				digiFunc->input = 0;
				digiFunc->secRemaining = 0.0;
			}
			// No else needed
		}
		else
		{
			// Turn off the function if not already off
			digiFunc->input = 0;
			digiFunc->wasReleased = 1;
			digiFunc->secTimePressed = -1.0;
			digiFunc->secRemaining = 0.0;
		}
	}

	// Finally, set the output value
	if( digiFunc->maintainToggle == 0 )
	{
		digiFunc->output = digiFunc->input;
		if((digiFunc->output == 1) && (digiFunc->releaseToReactivate == 1))
		{
			// Make sure the single set will be sent to the flight computer
			digiFunc->activeToSend = 1;
		}
	}
	else if( digiFunc->input)
	{
		// Toggle - flip it
		digiFunc->output = !digiFunc->output;
	}
	// else: toggle, but not activated - do nothing
}


void updateAnalogFunction( struct analogFunction_ref* analogFunc, struct input_ref* joy, struct controlInput_ref* mc  )
{
	// Only update whether it is active if in use
	if( analogFunc->inUse )
	{
		// Inputs range between -1.0 and 1.0
		if( analogFunc->useDigital )
		{
			if( (analogFunc->digitalMax >= 0) && (joy->joyInput[analogFunc->joystickId]->button[analogFunc->digitalMax] == 1))
			{
				analogFunc->input = analogFunc->maxValue;
			}
			else if( (analogFunc->digitalMin >= 0) && (joy->joyInput[analogFunc->joystickId]->button[analogFunc->digitalMin] == 1))
			{
				analogFunc->input = analogFunc->minValue;
			}
			else
			{
				// Not active, go to center value
				analogFunc->input = (analogFunc->maxValue - analogFunc->minValue) / 2.0F + analogFunc->minValue;
			}
		}
		else if( analogFunc->useHat )
		{
			if( (analogFunc->hatMax >= 0) && (joy->joyInput[analogFunc->joystickId]->hatswitch[analogFunc->hatMax] == 1))
			{
				analogFunc->input = analogFunc->maxValue;
			}
			else if( (analogFunc->hatMin >= 0) && (joy->joyInput[analogFunc->joystickId]->hatswitch[analogFunc->hatMin] == 1))
			{
				analogFunc->input = analogFunc->minValue;
			}
			else
			{
				// Not active, go to center value
				analogFunc->input = (analogFunc->maxValue - analogFunc->minValue) / 2.0F + analogFunc->minValue;
			}
		}
		else
		{
			// Normal analog map
			double signMult = (double)SIGN( joy->joyInput[analogFunc->joystickId]->joyAxis[analogFunc->axis] ) * ((analogFunc->reverseAxis) ? -1.0 : 1.0);
			// Fractional value between 0.0 and 1.0
			double fracVal = (MAX( ABS( joy->joyInput[analogFunc->joystickId]->joyAxis[analogFunc->axis] ) - analogFunc->deadBand, 0.0 )*
				signMult/MAX( 0.01, 1.0 - analogFunc->deadBand )) / 2.0 + 0.5;
			// Map the fractional value to the range
			analogFunc->input = (analogFunc->maxValue - analogFunc->minValue) * fracVal + analogFunc->minValue;
		}
	}

	// Finally, set the output value
	analogFunc->output = analogFunc->input;
}
