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

void Controller::update()
{
  //this->attitude_controller(sens, cmd);
  //this->altitude_controller(cmd);
  this->mixer();
}
/*
void Controller::attitude_controller(const sens_t& sens, const guidance_t& cmd)
{
  // attitude and rate control for roll and pitch
  this->roll_out  = P_ROLL_ANGLE*(FF_ROLL*cmd.ROLL - sens.euler[0]) - P_ROLL_RATE*sens.gyr[0] - D_ROLL_RATE*(sens.gyr[0] - this->last_rate[0]);
  this->pitch_out = P_PITCH_ANGLE*(FF_PITCH*cmd.PITCH - sens.euler[1]) - P_PITCH_RATE*sens.gyr[1] - D_PITCH_RATE*(sens.gyr[1] - this->last_rate[1]);

  this->yaw_out   = P_YAW_RATE * (cmd.YAW - sens.gyr[2]);

  for(uint8_t i = 0; i < 3; i++)
  {
    this->last_rate[i] = sens.gyr[i];
  }
}

void Controller::altitude_controller(const guidance_t& cmd)
{
  // throttle is passthrough since no altitude control (yet)
  this->thr_out = cmd.THR;
}
*/
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

 