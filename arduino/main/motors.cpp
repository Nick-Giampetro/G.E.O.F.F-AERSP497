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

#include "motors.h"
#include "Arduino.h"

Motors::Motors()
{

}

Motors::~Motors()
{

}

void Motors::init()
{
  for(uint8_t i=0; i<MOTOR_NUM; i++)
    motor[i].begin(20000.0, TIMER_SOURCE_DIV_1); // 20000 us is 50 Hz

  this->stop();
  
  // wait to make sure everything is set
  delay(1000); // it's okay to use delay in setup()
}

void Motors::calibrate()
{
  // ESC calibration: send max and min PWM to the ESC
  // Calibration must be done only once when you have a new ESC

  for(uint8_t i=0; i<MOTOR_NUM; i++) motor[i].pulseWidth_us(MAX_PWM_OUT);
  delay(5000);
  for(uint8_t i=0; i<MOTOR_NUM; i++) motor[i].pulseWidth_us(MIN_PWM_OUT);
  delay(5000);
}

void Motors::stop()
{
  for(uint8_t i=0; i<MOTOR_NUM; i++) motor[i].pulseWidth_us(MIN_PWM_OUT);
}

void Motors::update(int16_t pwm[MOTOR_NUM])
{
/*
   Serial.print(pwm[0]);  Serial.print(',');
   Serial.print(pwm[1]); Serial.print(',');
   Serial.print(pwm[2]); Serial.print(',');
   Serial.print(pwm[3]); Serial.print(',');

  pwm[0] = constrain(pwm[0], MIN_PWM_OUT, MAX_PWM_OUT); //thr
  pwm[1] = constrain(pwm[1], -PWM_LIMIT,PWM_LIMIT); //yaw
  pwm[2] = constrain(pwm[2], -PWM_LIMIT, PWM_LIMIT); //roll
  pwm[3] = constrain(pwm[3], -PWM_LIMIT,PWM_LIMIT);  //pitch
*/
  for(uint8_t i=0; i<MOTOR_NUM; i++) 
   {
     pwm[i] = constrain(pwm[i], MIN_PWM_OUT, MAX_PWM_OUT);
     //motor[i].pulseWidth_us(pwm[i]);
   }
    motor[0].pulseWidth_us(pwm[FRONT_RIGHT]); //D3  CCW
    motor[1].pulseWidth_us(pwm[FRONT_LEFT]); //D9  CW
    motor[2].pulseWidth_us(pwm[REAR_LEFT]); //D10 CCW
    motor[3].pulseWidth_us(pwm[REAR_RIGHT]); //D11 CW
}
void Motors::print()
{
  Serial.print("mc: "); 
  //Serial.print();  Serial.print(", "); 
  //Serial.print(a);   Serial.print(", "); 
  //Serial.print( b);  Serial.print(", "); 
  //Serial.print( c); Serial.print(", ");  
  //Serial.println(d);

}