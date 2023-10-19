#ifndef _RMAX_SENSORS_H_
#define _RMAX_SENSORS_H_

#if defined(__cplusplus)
extern "C"
{
#endif

	void readIMU(struct onboard_ref* ob);
	int sensors_close(struct onboard_ref* ob);
	void sensors_update(struct onboard_ref* ob);
	void mapControlsPWM(struct pwmServoMapping_ref* map,
		struct pwmServoMixing_ref* mix,
		struct pwmMTRServoMixing_ref* mmix,
		struct pwmServoMapping_ref* inmap,
		struct datalinkMessagePWM_ref* pwm,
		double* roll, double* pitch, double* yaw, double* thrust, double* throttle, double* nws, double* flaps, double* autopilot,
		short* spMode, short allowSPOverride);

	//for dji
	enum VerticalLogic
	{
		VERTICAL_VELOCITY = 0x00,
		VERTICAL_POSITION = 0x10,
		VERTICAL_THRUST = 0x20,
	};

	enum HorizontalLogic
	{
		HORIZONTAL_ANGLE = 0x00,
		HORIZONTAL_VELOCITY = 0x40,
		HORIZONTAL_POSITION = 0X80,
	};

	enum YawLogic
	{
		YAW_ANGLE = 0x00,
		YAW_RATE = 0x08
	};

	enum HorizontalCoordinate
	{
		HORIZONTAL_GROUND = 0x00,
		HORIZONTAL_BODY = 0x02
	};

	//! @version 3.1
	enum SmoothMode
	{
		SMOOTH_DISABLE = 0x00,
		SMOOTH_ENABLE = 0x01
	};

//#include "sickldmrsdriver.h"

#if defined(__cplusplus)
}
#endif


#endif // _RMAX_SENSORS_H_

