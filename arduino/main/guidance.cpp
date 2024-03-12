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

#include "guidance.h"

Guidance::Guidance()
{
  this->cmd.THR = 0;
  this->cmd.ROLL = 0;
  this->cmd.PITCH = 0;
  this->cmd.YAW = 0;
}

Guidance::~Guidance()
{
}

void Guidance::init()
{

}

void Guidance::update(const sens_t& sens, const state_t& state, const rc_t& rc)
{
  this->cmd.THR = rc.THR; // throttle is passthrough since no altitude control (yet)
  
  if(rc.ROLL > rc.ROLL_MID){
    this->cmd.ROLL = linear_map<float, int16_t>(rc.ROLL, rc.ROLL_MID, rc.ROLL_MAX, 0, ROLL_ANGLE_LIMIT);
  }
  else if(rc.ROLL < rc.ROLL_MID){
    this->cmd.ROLL = linear_map<float, int16_t>(rc.ROLL, rc.ROLL_MIN, rc.ROLL_MID, -ROLL_ANGLE_LIMIT, 0);
  }
  else{
    this->cmd.ROLL = 0;
  }
  
  if(rc.PITCH > rc.PITCH_MID){
    this->cmd.PITCH = linear_map<float, int16_t>(rc.PITCH, rc.PITCH_MID, rc.PITCH_MAX, 0, PITCH_ANGLE_LIMIT);
  }
  else if(rc.PITCH < rc.PITCH_MID){
    this->cmd.PITCH = linear_map<float, int16_t>(rc.PITCH, rc.PITCH_MIN, rc.PITCH_MID, -PITCH_ANGLE_LIMIT, 0);
  }
  else{
    this->cmd.PITCH = 0;
  }

  if(rc.YAW > rc.YAW_MID){
    this->cmd.YAW = linear_map<float, int16_t>(rc.YAW, rc.YAW_MID, rc.YAW_MAX, 0, YAW_RATE_LIMIT);
  }
  else if(rc.YAW < rc.YAW_MID){
    this->cmd.YAW = linear_map<float, int16_t>(rc.YAW, rc.YAW_MIN, rc.YAW_MID, -YAW_RATE_LIMIT, 0);
  }
  else{
    this->cmd.YAW = 0;
  }
}

void Guidance::print()
{
  Serial.print("Guidance: ");
  Serial.print(  this->cmd.THR);   Serial.print(", "); 
  Serial.print(  this->cmd.ROLL);  Serial.print(", "); 
  Serial.print(  this->cmd.PITCH); Serial.print(", ");  
  Serial.println(this->cmd.YAW);
}
