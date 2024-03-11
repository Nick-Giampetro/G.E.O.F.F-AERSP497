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

#ifndef CONTROLLER_H
#define CONTROLLER_H
#define NUM_MOTORS 4
#include "math_utils.h"
#include "pwm.h"
#include "motors.h"
#include "Arduino.h"

#define P_ROLL_ANGLE 0.9
#define P_PITCH_ANGLE 0.9

#define P_ROLL_RATE 0.12
#define P_PITCH_RATE 0.12
#define P_YAW_RATE 5.0

#define D_ROLL_RATE 1.0
#define D_PITCH_RATE 1.0

#define FF_ROLL 2
#define FF_PITCH 2
#define FF_YAW 2

class Controller
{
public:
	Controller();
	~Controller();

	void init();
  void update(const sens_t&, const state_t&, const guidance_t&);
  void print();

	int16_t pwm_out[MOTOR_NUM];

  float thr_out;
  float roll_out;
  float pitch_out;
  float yaw_out;

private:
  
  void attitude_controller(const sens_t&, const guidance_t&);
  void altitude_controller(const guidance_t&);
  void mixer();

  float last_rate[3];
};

#endif
