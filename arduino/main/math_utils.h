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

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <BasicLinearAlgebra.h>
#define Matrix BLA::Matrix

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958
#define MILLI2BASE 0.001
#define GRAVITY 9.81 // m/s^2

#define MOTOR_NUM 4
#define MIN_DUTY_CYCLE 125
#define MAX_DUTY_CYCLE 255
#define MIN_PWM_OUT 1000
#define MAX_PWM_OUT 2000
#define MIN_PWM_IN 1000
#define MID_PWM_IN 1500
#define MAX_PWM_IN 2000
#define ARM_DISARM_PWM_THRESHOLD 1800

#define ROLL_ANGLE_LIMIT 30 // deg
#define PITCH_ANGLE_LIMIT 30 // deg
#define YAW_RATE_LIMIT 60 // deg/s
#define PWM_LIMIT 500

using state_t = Matrix<16, 1>;
using quat_t = Matrix<4, 1>;
using vec_t = Matrix<3, 1>;
using mat3x3_t = Matrix<3, 3>;
using mat4x4_t = Matrix<4, 4>;
using euler_t = Matrix<3, 1>;

struct sens_t
{
  float gyr[3];
  float acc[3];
  float mag[3];
  float euler[3];
  float quat[4];
  float pos[3];
};

struct psens_t{
  float pos[3];
  float acc[3];
};

struct rc_t
{
  int16_t ROLL;
  int16_t PITCH;
  int16_t THR;
  int16_t YAW;
  int16_t AUX;
  int16_t AUX2;

  int16_t ROLL_MIN;
  int16_t PITCH_MIN;
  int16_t THR_MIN;
  int16_t YAW_MIN;
  int16_t AUX_MIN;
  int16_t AUX2_MIN;

  int16_t ROLL_MID;
  int16_t PITCH_MID;
  int16_t THR_MID;
  int16_t YAW_MID;
  int16_t AUX_MID;
  int16_t AUX2_MID;

  int16_t ROLL_MAX;
  int16_t PITCH_MAX;
  int16_t THR_MAX;
  int16_t YAW_MAX;
  int16_t AUX_MAX;
  int16_t AUX2_MAX;
};

struct guidance_t
{
  float THR;
  float ROLL;
  float PITCH;
  float YAW;
};

enum motor 
{
  FRONT_RIGHT,
  FRONT_LEFT,
  REAR_LEFT,
  REAR_RIGHT
};

enum rc 
{
  ROLL,
  PITCH,
  THR,
  YAW,
  AUX,
  AUX2
};

mat3x3_t rotation_from_quaternion(const quat_t&);
void normalize_quaternion(quat_t&);
void quat2euler( const quat_t& q, float& phi, float& theta, float& psi );

template <typename T>
T fisrt_order_euler_integration(T state, T derivative, const float& dt)
{
  return state + derivative * dt;
}

template <typename T>
T second_order_euler_integration(T state, T derivative1, T derivative2, const float& dt)
{
  return state + (derivative1 + derivative2) * (dt * 0.5);
}

template <typename T_out, typename T_in>
T_out linear_map(T_in x, T_in in_min, T_in in_max, T_out out_min, T_out out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif

