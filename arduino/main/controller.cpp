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

}

Controller::~Controller()
{
}

void Controller::init()
{

}

void Controller::update(uint16_t pwm[NUM_MOTORS])
{
  for(uint8_t i=0; i<NUM_MOTORS; i++) 
  {
    pwm[i] = constrain(pwm[i], MIN_PWM_OUT, MAX_PWM_OUT);
    //motor[i].pulseWidth_us(pwm[i]);
  }

 
  //this->roll_out = constrain(this->roll_out, -PWM_LIMIT, PWM_LIMIT);
  //this->pitch_out = constrain(this->pitch_out, -PWM_LIMIT, PWM_LIMIT);
  //this->yaw_out = constrain(this->yaw_out, -PWM_LIMIT, PWM_LIMIT);
  //this->thr_out = constrain(this->thr_out, MIN_PWM_OUT, MAX_PWM_OUT);

  //this->pwm_out[FRONT_RIGHT] = this->pwm[0] - this->pwm[2] + this->pwm[3] + this->pwm[1];
  motor[0].pulseWidth_us(pwm[0] - pwm[2] + pwm[3] + pwm[1]);
  //this->pwm_out[FRONT_LEFT]  = this->pwm[0] + this->pwm[2] + this->pwm[3] - this->pwm[1];
  motor[1].pulseWidth_us(pwm[0] + pwm[2] + pwm[3] - pwm[1]);
  //this->pwm_out[REAR_LEFT]   = this->pwm[0] + this->pwm[2] - this->pwm[3] + this->pwm[1];
  motor[2].pulseWidth_us(pwm[0] + pwm[2] - pwm[3] + pwm[1]);
  //this->pwm_out[REAR_RIGHT]  = this->pwm[0] - this->pwm[2] - this->pwm[3] - this->pwm[1];
  motor[3].pulseWidth_us(pwm[0] - pwm[2] - pwm[3] - pwm[1]);

  //for(uint8_t i=0; i<MOTOR_NUM; i++) this->pwm_out[i] = constrain(this->pwm_out[i], MIN_PWM_OUT, MAX_PWM_OUT);
}

/*void Controller::mixer()
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
}*/

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

 