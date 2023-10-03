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

#ifndef AERSP_RC_PILOT_H
#define AERSP_RC_PILOT_H

#include "Arduino.h"

#define RC_CHANS 6

#define MIN_PWM_IN 900
#define MAX_PWM_IN 2000
#define PWM_JUMP_LIMIT 990
#define PCINT_RX_BITS (1<<0),(1<<1),(1<<2),(1<<11),(1<<4),(1<<2)

static uint8_t RX_Pins[RC_CHANS] = {15, 16, 17, 6, 8, 1};
static uint16_t PCInt_RX_Pins[RC_CHANS] = {PCINT_RX_BITS};
static volatile uint16_t rcValue[RC_CHANS];
static volatile uint16_t edgeTime[RC_CHANS];

enum rc 
{
  ROLL,
  PITCH,
  THR,
  YAW,
  AUX,
  AUX2
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

class RC_PILOT
{
public:
  RC_PILOT();
  ~RC_PILOT();

  void init();
  void update();
  void print();

  rc_t rc_in;

private:
  int16_t rcData[RC_CHANS];
  int16_t rcData_previous[RC_CHANS];
  uint16_t io_ports[RC_CHANS] = {BSP_IO_PORT_00, BSP_IO_PORT_00, BSP_IO_PORT_00, BSP_IO_PORT_01, BSP_IO_PORT_03, BSP_IO_PORT_03};
};

#endif