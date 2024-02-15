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

#ifndef AERSP_SENSORS_H
#define AERSP_SENSORS_H

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958
#define MILLI2BASE 0.001

#define POZYX_GYR_SCALE 0.0625
#define POZYX_MAG_SCALE 0.0625
#define POZYX_EULER_SCALE 0.0625
#define POZYX_QUAT_SCALE 1.0/16384.0

#define GRAVITY 9.81

#define NUM_CALIBRATION 500
#define LOWPASS_WEIGHT 0.9

struct sens_t
{
  float gyr[3];
  float acc[3];
  float mag[3];
  float euler[3];
  float quat[4];
};

class Sensors
{
public:
  Sensors();
  ~Sensors();

  void init();
  void update();
  void print();

  sens_t data;

private:
  sens_t bias;
  bool calibration_flag;
};

#endif