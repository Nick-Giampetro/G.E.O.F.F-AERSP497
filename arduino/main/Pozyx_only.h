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

#include "Pozyx.h"
#include "Pozyx_definitions.h"
#include <Wire.h>
#include "math_utils.h"
#ifndef AERSP_PSENSORS_H
#define AERSP_PSENSORS_H

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958
#define MILLI2BASE 0.001

#define GRAVITY 9.81

#define NUM_CALIBRATION 500

#define LOWPASS_WEIGHT 0.0


class Pozyx_only
{
public:
  Pozyx_only();
  ~Pozyx_only();

  void init();
  void update();
  void print();
  void setAnchorsManual();

  psens_t data;

private:
  sens_t bias;
  bool calibration_flag;
  float duration, distance;
};

#endif