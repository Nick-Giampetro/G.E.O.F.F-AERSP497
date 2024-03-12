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

#include "navigation.h"

Navigation::Navigation()
{
  this->s.Fill(0);
  this->s_dot.Fill(0);
}

Navigation::~Navigation() {}

void Navigation::init()
{
  this->s(0) = 1; // q0
}

void Navigation::update(const sens_t& sens, const float& dt)
{
  /* state propagation */
  this->process_model(sens, this->s, this->s_dot);
  this->s_predicted = fisrt_order_euler_integration<state_t>(this->s, this->s_dot, dt);

  /* Below is 2nd-order Euler Integration, but Arduino is kind of slow */
  // state_t s_euler, s_dot_euler;
  // this->process_model(sens, this->s, this->s_dot);
  // s_euler = fisrt_order_euler_integration<state_t>(this->s, this->s_dot, dt);
  // this->process_model(sens, s_euler, s_dot_euler);
  // this->s_predicted = second_order_euler_integration<state_t>(this->s, this->s_dot, s_dot_euler, dt);

  /* covariance propagation */

  /* compute Kalman gain */

  /* measurement update */
  this->s = this->s_predicted;

  /* covariance update */
}

void Navigation::process_model(const sens_t& sens, const state_t& state, state_t& state_dot)
{
  quat_t q = {state(0), state(1), state(2), state(3)}; // quaternion
  vec_t ba = {state(10), state(11), state(12)}; // acc bias
  vec_t bw = {state(13), state(14), state(15)}; // gyro bias
  vec_t a = {sens.acc[0], sens.acc[1], sens.acc[2]}; // acceleration  
  vec_t g = {0, 0, -GRAVITY};

  normalize_quaternion(q);
  mat3x3_t T = rotation_from_quaternion(q);

  mat4x4_t f;
  f.Fill(0);
  f(0,1) = -(sens.gyr[0] - bw(0)) * 0.5;
  f(0,2) = -(sens.gyr[1] - bw(1)) * 0.5;
  f(0,3) = -(sens.gyr[2] - bw(2)) * 0.5;

  f(1,0) =  (sens.gyr[0] - bw(0)) * 0.5;
  f(1,2) =  (sens.gyr[2] - bw(2)) * 0.5;
  f(1,3) = -(sens.gyr[1] - bw(1)) * 0.5;

  f(2,0) =  (sens.gyr[1] - bw(1)) * 0.5;
  f(2,1) = -(sens.gyr[2] - bw(2)) * 0.5;
  f(2,3) =  (sens.gyr[0] - bw(0)) * 0.5;

  f(3,0) =  (sens.gyr[2] - bw(2)) * 0.5;
  f(3,1) =  (sens.gyr[1] - bw(1)) * 0.5;
  f(3,2) = -(sens.gyr[0] - bw(0)) * 0.5;

  quat_t q_dot = f * q;
  vec_t v_dot = T * (a - ba) + g;

  state_dot(0) = q_dot(0);
  state_dot(1) = q_dot(1);
  state_dot(2) = q_dot(2);
  state_dot(3) = q_dot(3);
  state_dot(4) = state(7);
  state_dot(5) = state(8);
  state_dot(6) = state(9);
  state_dot(7) = v_dot(0);
  state_dot(8) = v_dot(1);
  state_dot(9) = v_dot(2);

  quat2euler(q, this->angles(0), this->angles(1), this->angles(2));
}

void Navigation::print()
{
  // Serial.print("s: ");
  // Serial << this->s;
  // Serial.println();
  // Serial.print("s_dot: ");
  // Serial << this->s_dot;
  // Serial.println();

  //Serial << this->angles * RAD2DEG;
  Serial.println();
}