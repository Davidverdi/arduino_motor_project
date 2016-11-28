/* 
28th November 2016

This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

Here we are just using the setup loop in an attempt to set the motor to turn
at a constant speed.
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(254);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:

}
