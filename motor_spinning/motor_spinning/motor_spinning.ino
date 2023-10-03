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

/*
 Motors/ESCs Spinning Direction:
  - Front Right -> Pin 3,  CCW
  - Back Right  -> Pin 5,  CW
  - Back Left   -> Pin 9, CCW
  - Front Left  -> Pin 10, CW
*/

#include "motors.h"
#include "rc_pilot.h"

RC_PILOT rc;

Motors motors;

uint16_t i, kill;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motors.init();
  rc.init();
  // motors.calibrate();
  pinMode(LED_BUILTIN, OUTPUT);
  
  

}

int safe = 0;

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  rc.update();
  rc.print();

  i = rc.rc_in.THR;
  kill = rc.rc_in.AUX;

  uint16_t pwm[4] = {i,i,i,i};

  if (i < 1005 && safe == 1){
    safe = 0;
  }

  if (kill > 1500 && safe == 0){
    motors.update(pwm);
  }
  else{
    pwm[0] = 1000;
    pwm[1] = 1000;
    pwm[2] = 1000;
    pwm[3] = 1000;
    motors.update(pwm);
    safe = 1;
  }

  
  
  //delay(1); // just for testing a motor
  
}
