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

#ifndef AERSP_MOTORS_H
#define AERSP_MOTORS_H

#include "pwm.h"

#define NUM_MOTORS 4
#define MIN_PWM_OUT 900
#define MAX_PWM_OUT 1900

class Motors
{
public:
  Motors();
  ~Motors();

  void init();
  void calibrate();
  void stop();
  void update(uint16_t[NUM_MOTORS]);

private:
  PwmOut motor[4] = {PwmOut(3), PwmOut(5), PwmOut(9), PwmOut(10)};
};

#endif