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

#ifndef NAVIGATION_H
#define NAVIGATION_H

//#include "math_utils.h"
#include "Arduino.h"



//state = [q0,q1,q2,q3,x,y,z,u,v,w,bax,bay,baz,bwx,bwy,bwz]


class Navigation
{
public:
  Navigation();
  ~Navigation();

  void init();
  void update(const sens_t&, const float&);

  void print();

  state_t s; // x^hat+
  state_t s_dot; // x^hat_dot
  state_t s_predicted; // x^hat-
  
  euler_t angles;

private:
  void process_model(const sens_t&, const state_t&, state_t&);
};

#endif