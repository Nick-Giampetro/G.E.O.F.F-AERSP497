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

#include "controller.h"

#ifndef C_PI
#define C_PI 3.14159265358979323846264338327950288419716939937511
#endif

#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

Controller::Controller()
{
	for ( auto i = 0; i < MOTOR_NUM; i++ )
	{
		this->pwm_out[i] = MIN_PWM_OUT;
	}

  thr_out = 0;
  roll_out = 0;
  pitch_out = 0;
  yaw_out = 0;
}

Controller::~Controller()
{
}

void Controller::init()
{

}

void Controller::update(const sens_t& sens, const state_t& state, const guidance_t& cmd)
{
  this->attitude_controller(sens, cmd);
  this->altitude_controller(cmd);
  this->mixer();
}

void Controller::attitude_controller(const sens_t& sens, const guidance_t& cmd)
{
  // attitude and rate control for roll and pitch
  this->roll_out  = P_ROLL_ANGLE * (cmd.ROLL/ROLL_ANGLE_LIMIT - sens.euler[0]) - P_ROLL_RATE * sens.gyr[0];
  this->pitch_out = P_PITCH_ANGLE *(cmd.PITCH/PITCH_ANGLE_LIMIT - sens.euler[1]) - P_PITCH_RATE * sens.gyr[1];

  this->yaw_out   = P_YAW_ANGLE * (float)this->hmodRad(cmd.YAW  - sens.euler[2]) - P_YAW_RATE * sens.gyr[2];

  for(uint8_t i = 0; i < 3; i++)
  {
    this->last_rate[i] = sens.gyr[i];
  }
}

void Controller::altitude_controller(const guidance_t& cmd)
{
  // throttle is passthrough since no altitude control (yet)
  this->thr_out = cmd.THR;

  // working on the controller but need nav
  
  // this->Altitude_integral += (float)cntrl->work->dt * (- posDes_z - nav_p_z);
  // this->Altitude_integral = LIMIT(this->Altitude_integral, -1000, 1000);
  // this->thr_out = - P_ALTITUDE_POS * ( - posDes_z - nav_p_z)
	//                 - P_ALTITUDE_VEL * (-nav_v_z)
	//                 - P_ALTITUDE_INT * this->Altitude_integral ;
}

void Controller::mixer()
{
  this->roll_out = constrain(this->roll_out, -PWM_LIMIT, PWM_LIMIT);
  this->pitch_out = constrain(this->pitch_out, -PWM_LIMIT, PWM_LIMIT);
  this->yaw_out = constrain(this->yaw_out, -PWM_LIMIT, PWM_LIMIT);
  this->thr_out = constrain(this->thr_out, MIN_PWM_OUT, MAX_PWM_OUT);

  this->pwm_out[FRONT_RIGHT] = this->thr_out - this->roll_out + this->pitch_out + this->yaw_out;
  this->pwm_out[FRONT_LEFT]  = this->thr_out + this->roll_out + this->pitch_out - this->yaw_out;
  this->pwm_out[REAR_LEFT]   = this->thr_out + this->roll_out - this->pitch_out + this->yaw_out;
  this->pwm_out[REAR_RIGHT]  = this->thr_out - this->roll_out - this->pitch_out - this->yaw_out;

  for(uint8_t i=0; i<MOTOR_NUM; i++) this->pwm_out[i] = constrain(this->pwm_out[i], MIN_PWM_OUT, MAX_PWM_OUT);
}

void Controller::print()
{
//  Serial.print("cntrl out: ");
//  Serial.print(  this->thr_out);   Serial.print(", "); 
//  Serial.print(  this->roll_out);  Serial.print(", "); 
//  Serial.print(  this->pitch_out); Serial.print(", ");  
//  Serial.println(this->yaw_out);

  Serial.print("pwm out: ");
  Serial.print(  this->pwm_out[FRONT_RIGHT]); Serial.print(", "); 
  Serial.print(  this->pwm_out[FRONT_LEFT]);  Serial.print(", "); 
  Serial.print(  this->pwm_out[REAR_LEFT]);   Serial.print(", ");  
  Serial.println(this->pwm_out[REAR_RIGHT]);
}

double Controller::hmodRad(double h) {

	double dh;
	int i;

	if (h > 0)
		i = (int)(h / (2 * C_PI) + 0.5);
	else
		i = (int)(h / (2 * C_PI) - 0.5);
	dh = h - C_PI * 2 * i;

	return dh;
}