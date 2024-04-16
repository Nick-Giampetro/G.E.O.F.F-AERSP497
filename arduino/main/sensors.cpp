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

#define trigPin 12
#define echoPin 11

uint16_t remote_id = 0x6838;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = false;                         // set this to true to output data for the processing sketch

const uint8_t num_anchors = 6;                                    // the number of anchors
uint16_t anchors[num_anchors] =  {0x1251, 0x125e, 0x117a, 0x113f, 0x122e, 1148};     // {BL,BR,TLT,TLB,TRT,TRB} the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] = {-3262,  2834,   -3262,  -3262,  2834,   2834};               // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] = {921,    921,    5595,   5595,   5595,   5595};                  // anchor y-coordinates in mm
int32_t heights[num_anchors] =   {7,      7,      7,      2616,   2616,   8};              // anchor z-coordinates in mm

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 1000;                                  // height of device, required in 2.5D positioning

Sensors::Sensors()
{
  for(uint8_t i = 0; i < 3; i++)
  {
    this->data.gyr[i]   = 0;
    this->data.acc[i]   = 0;
    this->data.mag[i]   = 0;
    this->data.euler[i] = 0;
    this->data.quat[i]  = 0;
    this->data.pos[i]   = 0;

    this->bias.gyr[i]   = 0;
    this->bias.acc[i]   = 0;
    this->bias.mag[i]   = 0;
    this->bias.euler[i] = 0;
    this->bias.quat[i]  = 0;
    this->bias.pos[i]   = 0;
  }
  this->data.quat[0] = 1;
  this->calibration_flag = 0;
}

Sensors::~Sensors() {}

void Sensors::init()
{
  Serial.begin(115200);

  if(!remote)
  {
  remote_id = NULL;
  }

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

  // clear all previous pozyx devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setAnchorsManual();
  // sets the positioning algorithm
  Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);
  
  this->calibration_flag = 1;

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

long duration, inches, cm;
long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void Sensors::update()
{

  // digitalWrite(trigPin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPin, LOW);

  // this->duration = pulseIn(echoPin, HIGH);
  // this->distance = (this->duration*.0343)/2;
  // Serial.print("Distance: ");
  // Serial.println(this->distance);


  sensor_raw_t sensor_raw;
  if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 10) == POZYX_SUCCESS){
    Pozyx.getRawSensorData(&sensor_raw);
  }else{
    uint8_t interrupt_status = 0;
    Pozyx.getInterruptStatus(&interrupt_status);
    return;
  }
  
  // pozyx stuffs
  coordinates_t position;
  int status;
  if(remote){
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
  }else{
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  }

  // gathering 
  this->data.pos[0] = position.x;
  this->data.pos[1] = position.y;
  this->data.pos[2] = position.z;

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
  // Serial.print(this->data.euler[0]);
  // Serial.print(",");
  // Serial.print(this->data.euler[1]);
  // Serial.print(",");
  // Serial.print(this->data.euler[2]);
Serial.print("x(mm): ");
Serial.print(this->data.pos[0]);
Serial.print(", y(mm): ");
Serial.print(this->data.pos[1]);
Serial.print(", z(mm): ");
Serial.print(this->data.pos[2]);
  Serial.println();
}

// function to manually set the anchor coordinates
void Sensors::setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}