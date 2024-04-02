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
#include "sensors.h"
#include "motors.h"
#include "rc_pilot.h"
#include "controller.h"
#include "guidance.h"
#include "navigation.h"
#include <Servo.h>

#define NUM_MOTORS 4
#define MIN_PWM_OUT 900
#define MAX_PWM_OUT 1900
#define MIN_PWM_OUT_O 900
#define MAX_PWM_OUT_O 1900

Sensors sens;
RC_PILOT rc;
Motors motors;
Servo myservo;
Controller cntrl;
Navigation nav;
Guidance gd;



int pos = 0;
uint16_t i, thr, yaw, roll, pitch, kill, servo;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motors.init();
  sens.init();
  nav.init();
  rc.init();
  gd.init();
  cntrl.init();
  
  //myservo.attach(11);
  // motors.calibrate();
  pinMode(LED_BUILTIN, OUTPUT);

  float dt, cTime, lTime = 0 ;

}

int safe = 0;

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));


  //cTime = millis();
  //dt = lTime - cTime;
  //lTime = cTime;

  rc.update();

  //rc.print();
  //motors.print();
  thr = rc.rc_in.THR;
  yaw = rc.rc_in.YAW;
  roll = rc.rc_in.ROLL;
  pitch = rc.rc_in.PITCH;
  kill = rc.rc_in.AUX;
  servo = rc.rc_in.AUX2;

  int16_t pwm[4] = {thr,yaw,roll,pitch};
  
  sens.update();
  gd.update(sens.data,nav.s,rc.rc_in);
  // gd.print();
  cntrl.update(sens.data, nav.s, gd.cmd);
  //cntrl.print();
  // sens.print();
  // rc.print();


  if (thr < 1005 && safe == 1){
    safe = 0;
  }

  if (kill > 1500 && safe == 0){
  
    //motors.update(pwm);
    //Serial.println(motors.limit(pwm[0] - pwm[2] + pwm[3] + pwm[1]));
    if (thr > 1010)
      motors.update(cntrl.pwm_out);
    else
      motors.stop();

  }
  else{
    
    // Serial.println(safe);
    motors.stop();
  
    safe = 1;
  }

  
  
  //delay(1); // just for testing a motor
  
}
