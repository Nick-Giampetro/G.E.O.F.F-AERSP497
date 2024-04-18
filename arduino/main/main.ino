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

const int trigPin = 12;
const int echoPin = 11;
// defines variables
long duration;
float distance;
bool safe = false ;
int pos = 0;
uint16_t i, thr, yaw, roll, pitch, serv, multi;





void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  sens.init();
  nav.init();
  rc.init();
  gd.init();
  cntrl.init();
  motors.init();
  cntrl.altitude_hold(false);
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  myservo.attach(13);
  // motors.calibrate();
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  rc.update();
  thr = rc.rc_in.THR;
  yaw = rc.rc_in.YAW;
  roll = rc.rc_in.ROLL;
  pitch = rc.rc_in.PITCH;
  serv = rc.rc_in.AUX;
  multi = rc.rc_in.AUX2;

  int16_t pwm[4] = {thr,yaw,roll,pitch};
  
  sens.update();
  gd.update(sens.data,nav.s,rc.rc_in);

  cntrl.update(sens.data, nav.s, gd.cmd);
  //cntrl.print();
  //sens.print();
  //rc.print();
  //motors.print();
  // gd.print();


  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin, HIGH);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  cntrl.distance(distance,sens.data);

  if(safe == false && ((thr < 1010 && !cntrl.get_mode())))
    safe = true ;

  if (multi > 1450 && safe) {
    if ((thr > 1010 && !cntrl.get_mode()) || cntrl.get_mode()){
     motors.update(cntrl.pwm_out);
    }
    else
     motors.stop();
  }
  else {
    safe = false;
    motors.stop();
  }

  if(multi > 1450 && multi < 1550)
    cntrl.altitude_hold(true);
  else if(multi > 1950 && multi <= 2000) 
    cntrl.altitude_hold(true);
  else
    cntrl.altitude_hold(false);

  if( serv > 1500 )
    myservo.write(0); 
  else
    myservo.write(180); 
}
