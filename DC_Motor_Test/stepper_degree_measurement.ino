#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int steps_per_rev = 200;   // Determine Experimentally
int stepper_port = 2;      // Port 2, which is M3 and M4
int stepper_speed = 10;    // 10 rpm

Adafruit_MotorShield AFMS = Adafruit_MotorShield ()
Adafruit_StepperMotor *stepper = AFMS.getStepper(steps_per_rev, stepper_port);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  
  count_steps();
}

void loop() {
  // put your main code here, to run repeatedly:

}

  
void count_steps() {
  //Allows us to experimentally find the number of steps per revolution
  int total = 0;
  int i = 0;
  while(i < 300)  //300 should be enough steps to make one revolution
  {
    stepper->step(1);
    total++;
    Serial.print("steps:");
    Serial.println(total);
    delay(100);
  }
