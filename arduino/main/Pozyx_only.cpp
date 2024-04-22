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

#include "Pozyx_only.h"
#include "sensors.h"
#include "Pozyx.h"
#include "Pozyx_definitions.h"
#include <Wire.h>



uint16_t P_remote_id = 0x6838;                            // set this to the ID of the remote device
bool P_remote= false;                                    // set this to true to use the remote ID

boolean P_use_processing = false;                         // set this to true to output data for the processing sketch

const uint8_t P_num_anchors = 6;                                    // the number of anchors
uint16_t P_anchors[P_num_anchors] =  {0x1251, 0x125e, 0x117a, 0x113f, 0x122e, 1148};     // {BL,BR,TLT,TLB,TRT,TRB} the network id of the anchors: change these to the network ids of your anchors.
int32_t P_anchors_x[P_num_anchors] = {-3262,  2834,   -3262,  -3262,  2834,   2834};               // anchor x-coorindates in mm
int32_t P_anchors_y[P_num_anchors] = {921,    921,    5595,   5595,   5595,   5595};                  // anchor y-coordinates in mm
int32_t P_heights[P_num_anchors] =   {7,      7,      7,      2616,   2616,   8};              // anchor z-coordinates in mm

uint8_t P_algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
uint8_t P_dimension = POZYX_3D;                           // positioning dimension
int32_t P_height = 1000;                                  // height of device, required in 2.5D positioning

Pozyx_only::Pozyx_only()
{
  for(uint8_t i = 0; i < 3; i++)
  {
    this->data.acc[i]   = 0;
    this->data.pos[i]   = 0;

    this->bias.acc[i]   = 0;
    this->bias.pos[i]   = 0;
  }
  this->calibration_flag = 0;
 
}

Pozyx_only::~Pozyx_only() {
}

void Pozyx_only::init()
{
  Serial.begin(115200);

  if(!P_remote)
  {
  P_remote_id = NULL;
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
      this->bias.acc[j] += this->data.acc[j];
      this->bias.pos[j] += this->data.pos[j];
    }
  }
  for(uint8_t i = 0; i < 3; i++)
  {
    this->bias.acc[i] /= NUM_CALIBRATION;
    this->bias.pos[i] /= NUM_CALIBRATION;

    this->data.acc[i] = 0;
    this->data.pos[i] = 0;
  }


  // clear all previous pozyx devices in the device list
  Pozyx.clearDevices(P_remote_id);
  // sets the anchor manually
  setAnchorsManual();
  // sets the positioning algorithm
  Pozyx.setPositionAlgorithm(P_algorithm, P_dimension, P_remote_id);
  
  this->calibration_flag = 1;

}


void Pozyx_only::update()
{

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
  if(P_remote){
    status = Pozyx.doRemotePositioning(P_remote_id, &position, P_dimension, P_height, P_algorithm);
  }else{
    status = Pozyx.doPositioning(&position, P_dimension, P_height, P_algorithm);
  }

  // gathering 
  this->data.pos[0] = position.x;
  this->data.pos[1] = position.y;
  this->data.pos[2] = position.z;


  //Linear acceleration (No idea what type of data is this)
  this->data.acc[0] = sensor_raw.linear_acceleration[0];
  this->data.acc[1] = sensor_raw.linear_acceleration[1];
  this->data.acc[2] = sensor_raw.linear_acceleration[2];
  
}

void Pozyx_only::print()
{
  // Serial.print(this->data.gyr[0]);
  // Serial.print(",");
  // Serial.print(this->data.gyr[1]);
  // Serial.print(",");
  // Serial.print(this->data.gyr[2]);
  // Serial.print(",");
  Serial.print(this->data.acc[0]);
  Serial.print(",");
  Serial.print(this->data.acc[1]);
  Serial.print(",");
  Serial.print(this->data.acc[2]);
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
void Pozyx_only::setAnchorsManual(){
  for(int i = 0; i < P_num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = P_anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = P_anchors_x[i];
    anchor.pos.y = P_anchors_y[i];
    anchor.pos.z = P_heights[i];
    Pozyx.addDevice(anchor, P_remote_id);
  }
  if (P_num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, P_num_anchors, P_remote_id);
  }
}