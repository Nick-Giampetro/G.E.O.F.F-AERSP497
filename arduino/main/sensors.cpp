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

#include "sensors.h"

#include "Pozyx.h"
#include "Pozyx_definitions.h"
#include <Wire.h>

#define POZYX_GYR_SCALE 0.0625
#define POZYX_MAG_SCALE 0.0625
#define POZYX_EULER_SCALE 0.0625
#define POZYX_QUAT_SCALE 1.0/16384.0

Sensors::Sensors()
{
  for(uint8_t i = 0; i < 3; i++)
  {
    this->data.gyr[i]   = 0;
    this->data.acc[i]   = 0;
    this->data.mag[i]   = 0;
    this->data.euler[i] = 0;
    this->data.quat[i]  = 0;

    this->bias.gyr[i]   = 0;
    this->bias.acc[i]   = 0;
    this->bias.mag[i]   = 0;
    this->bias.euler[i] = 0;
    this->bias.quat[i]  = 0;
  }
  this->data.quat[0] = 1;
  this->calibration_flag = 0;
}

Sensors::~Sensors() {}

void Sensors::init()
{
  Serial.println("Init");
  if(Pozyx.begin(true, MODE_INTERRUPT, POZYX_INT_MASK_IMU, 0x02) == POZYX_FAILURE)
  {
    Serial.println("ERROR: Unable to connect to POZYX shield");
    Serial.println("Reset required");
    delay(100);
    // abort();
  }
  for(unsigned long i = 0; i < NUM_CALIBRATION; i++)
  {
    this->update();
    for(uint8_t j = 0; j < 3; j++)
    {
      this->bias.gyr[j]   += this->data.gyr[j];
      this->bias.euler[j] += this->data.euler[j];
    }
  }
  for(uint8_t i = 0; i < 3; i++)
  {
    this->bias.gyr[i]   /= NUM_CALIBRATION;
    this->bias.euler[i] /= NUM_CALIBRATION;

    this->data.gyr[i]   = 0;
    this->data.euler[i] = 0;
  }
  this->calibration_flag = 1;
}

void Sensors::update()
{
  sensor_raw_t sensor_raw;
  if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 10) == POZYX_SUCCESS){
    Pozyx.getRawSensorData(&sensor_raw);
  }else{
    uint8_t interrupt_status = 0;
    Pozyx.getInterruptStatus(&interrupt_status);
    return;
  }
  
  // YPR to RPY and NED
  this->data.euler[0] = sensor_raw.euler_angles[1] * POZYX_EULER_SCALE; // convert to deg
  this->data.euler[1] = sensor_raw.euler_angles[2] * POZYX_EULER_SCALE; // convert to deg
  this->data.euler[2] = sensor_raw.euler_angles[0] * POZYX_EULER_SCALE; // convert to deg

  if(this->calibration_flag) // regular reading
  {
    float temp_gyr[3];
    temp_gyr[0] = sensor_raw.angular_vel[1] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    temp_gyr[1] = sensor_raw.angular_vel[0] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    temp_gyr[2] = sensor_raw.angular_vel[2] * POZYX_GYR_SCALE * -1.0; // convert to deg/s

    for(uint8_t i = 0; i < 3; i++)
    {
      this->data.euler[i] -= this->bias.euler[i];
      temp_gyr[i]         -= this->bias.gyr[i];
    }

    for(uint8_t i = 0; i < 3; i++)
    {
      this->data.gyr[i] = LOWPASS_WEIGHT*this->data.gyr[i] + (1-LOWPASS_WEIGHT)*temp_gyr[i]; // low-pass filter
    }    
  }
  else // calibration step
  {
    this->data.gyr[0] = sensor_raw.angular_vel[1] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    this->data.gyr[1] = sensor_raw.angular_vel[0] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
    this->data.gyr[2] = sensor_raw.angular_vel[2] * POZYX_GYR_SCALE * -1.0; // convert to deg/s
  }
}

void Sensors::print()
{
  Serial.print(this->data.gyr[0]);
  Serial.print(",");
  Serial.print(this->data.gyr[1]);
  Serial.print(",");
  Serial.print(this->data.gyr[2]);
  Serial.print(",");
//  Serial.print(this->data.acc[0]);
//  Serial.print(",");
//  Serial.print(this->data.acc[1]);
//  Serial.print(",");
//  Serial.print(this->data.acc[2]);
//  Serial.print(",");
//  Serial.print(this->data.mag[0]);
//  Serial.print(",");
//  Serial.print(this->data.mag[1]);
//  Serial.print(",");
//  Serial.print(this->data.mag[2]);
//  Serial.print(",");
  Serial.print(this->data.euler[0]);
  Serial.print(",");
  Serial.print(this->data.euler[1]);
  Serial.print(",");
  Serial.print(this->data.euler[2]);
  Serial.println();
}