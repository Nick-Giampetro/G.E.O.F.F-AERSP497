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
#include <cmath>

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
  lTime = 0 ;

}

Controller::~Controller()
{
}

void Controller::init()
{

}

void Controller::update(const sens_t& sens, const state_t& state, const guidance_t& cmd)
{
  this->cTime = millis();
  this->dt = (this->cTime - this->lTime)/1000;
  this->lTime = this->cTime;

  //Serial.println(dt);

  //this->velocity_controller(sens, cmd)
  this->attitude_controller(sens, cmd);
  this->altitude_controller(sens, cmd);
  this->mixer();
}

void Controller::velocity_controller(const sens_t& sens, const guidance_t& cmd)
{

  // float crtc [2] = {0,0} ;

  // float posError_Hdg[2] = {+cntrl->posError[0] * cos(sens.euler[2]) + cntrl->posError[1] * sin(sens.euler[1]) + crtc[0], 
  //                           -cntrl->posError[0] * sin(sens.euler[2]) + cntrl->posError[1] * cos(sens.euler[2]) + crtc[1]}; 
  
  // float velError_Hdg[0]  {+nav_v_x * cos(sens.euler[2]) + nav_v_y * sin(sens.euler[2]),
  //                         -nav_v_x * sin(sens.euler[2]) + nav_v_y * cos(sens.euler[2])};


  // float VEL_X_CMD = LIMIT((P_X_VEL * POS_X_ERROR_HDG)) / P_X_POS, -MAX_SPEED , MAX_SPEED);
  // float VEL_Y_CMD = LIMIT((P_Y_VEL* POS_Y_ERROR_HDG)) / P_Y_POS, -MAX_SPEED , MAX_SPEED);

  // CMD_ROLL = +(cntrl->KDol[1] * (VEL_Y_CMD - cntrl->velError_Hdg[1]));
  // CMD_PITCH = -(cntrl->KDol[0] * (VEL_X_CMD - cntrl->velError_Hdg[0]));

  // this->roll_out  = P_ROLL_ANGLE * (CMD_ROLL + sens.euler[0]) + P_ROLL_RATE * sens.gyr[0] + D_ROLL_RATE*(sens.gyr[0] - this->last_rate[0]);
  // this->pitch_out = - P_PITCH_ANGLE * (CMD_PITCH - sens.euler[1]) + P_PITCH_RATE * sens.gyr[1] + D_PITCH_RATE*(sens.gyr[1] - this->last_rate[1]);

  // this->yaw_out   = P_YAW_ANGLE * (float)this->hmodRad(cmd.YAW  - sens.euler[2]) - P_YAW_RATE * sens.gyr[2];
}

void Controller::attitude_controller(const sens_t& sens, const guidance_t& cmd)
{
  // attitude and rate control for roll and pitch
  // this->roll_out  = P_ROLL_ANGLE * (cmd.ROLL/ROLL_ANGLE_LIMIT - sens.euler[0]) - P_ROLL_RATE * sens.gyr[0];
  // this->pitch_out = P_PITCH_ANGLE *(cmd.PITCH/PITCH_ANGLE_LIMIT - sens.euler[1]) - P_PITCH_RATE * sens.gyr[1];

  //Serial.print(sens.euler[0]) ; Serial.print(", ") , Serial.println(sens.euler[1]) ;
  //Serial.print(sens.gyr[0]) ; Serial.print(", ") , Serial.println(sens.gyr[1]) ;
  //Serial.println(sens.gyr[2]) ;


  // this->roll_out  = P_ROLL_ANGLE*(FF_ROLL*cmd.ROLL - sens.euler[0]) - P_ROLL_RATE*sens.gyr[0] - D_ROLL_RATE*(sens.gyr[0] - this->last_rate[0]);
  // this->pitch_out = P_PITCH_ANGLE*(FF_PITCH*cmd.PITCH - sens.euler[1]) - P_PITCH_RATE*sens.gyr[1] - D_PITCH_RATE*(sens.gyr[1] - this->last_rate[1]);


  this->Attitude_integral[0] += dt*(cmd.ROLL + sens.euler[0]);
  this->Attitude_integral[0] = LIMIT(this->Attitude_integral[0], -100, 100);
  this->roll_out  = P_ROLL_ANGLE * (cmd.ROLL + sens.euler[0]) + P_ROLL_RATE * sens.gyr[0] + P_ROLL_INT * this->Attitude_integral[0] + ROLL_BIAS;
  
  this->Attitude_integral[1] += dt*(cmd.PITCH - sens.euler[1]);
  this->Attitude_integral[1] = LIMIT(this->Attitude_integral[1], -100, 100);
  this->pitch_out = - P_PITCH_ANGLE * (cmd.PITCH - sens.euler[1]) + P_PITCH_RATE * sens.gyr[1] - P_PITCH_INT * this->Attitude_integral[1] + PITCH_BIAS;


  //Serial.println(this->roll_out) ;
  //Serial.println(this->pitch_out) ;
  this->yaw_out = P_YAW_RATE * (cmd.YAW - sens.gyr[2]);

  // Serial.print(cmd.YAW ) ;
  // Serial.print(",") ;
  // Serial.print(sens.gyr[2]) ;
  //  Serial.println() ;
  //Serial.println(cmd.YAW) ;
 
  for(uint8_t i = 0; i < 3; i++)
  {
    this->last_rate[i] = sens.gyr[i];
  }
      for(uint8_t i = 0; i < 3; i++)
  {
    this->last_acc[i] = sens.acc[i];
  }
  this->lDist = this->dist ;
}

void Controller::altitude_hold(bool nm){
  this->alt_mode = nm ;
}

bool Controller::get_mode(){
  return this->alt_mode ;
}

void Controller::reset_integral(){
  this->Altitude_integral = 0 ;
}

void Controller::altitude_controller(const sens_t& sens, const guidance_t& cmd)
{  
  if(this->alt_mode) {
  float posDes_z = 3.2 ;

  // working on the controller but need nav
  
  this->Altitude_integral += this->dt * (- posDes_z - this->dist );
  this->Altitude_integral = LIMIT(this->Altitude_integral, -50, 50);
  this->thr_out = - P_ALTITUDE_POS * ( - posDes_z - (( this->dist + this->lDist ) / 2 ))
	                - P_ALTITUDE_VEL * ((sens.acc[2] + this->last_acc[2])/2) * dt  // need to fix this
	                - P_ALTITUDE_INT * this->Altitude_integral 
                  + ALTITUDE_BIAS ;

  this->thr_out = LIMIT(this->thr_out,1000,2000);
  
  }
  else
    this->thr_out = cmd.THR;

  double zVel = ((sens.acc[2] + this->last_acc[2])/2) * dt ;
  // Serial.println(zVel);
  // Serial.println(this->thr_out) ;

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
float Controller::distance(float d,const sens_t& sens)
{
  this->dist = (d * -0.032808399) ;
  Serial.print("Distance (ft): ");
  Serial.println(dist);
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