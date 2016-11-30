/* ARDUINO PROJECT
 
 Aim: Detect the direction of brightest light
 and turn a solar cell to face that direction.

 Date: 28th November 2016
 Contributors:
   Kevin Baquero
   David Verdi
   Adelaide Young
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield ();

//Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *stepper = AFMS.getStepper(200,2);

//Connect the DC motor to port M2
Adafruit_DCMotor *DCMotor = AFMS.getMotor(2)


void setup() {
  // put your setup code here, to run once:
 /* Set stepper, potientiometer and DC motor(?)
  *  to zero position
  * Start stepper motor.
  * Read signal from photo-resistor.
  */
  Serial.begin(9600);        //set up Serial library at 9600 bps
  Serial.println("Light Aiming Beam (L.A.B.)");

  AFMS.begin();        // create with the default frequency 1.6kHz

  //This sets the motor and stepper speeds.
  DCMotor->setSpeed(150)
  stepper->setSpeed(20)        // 10 rpm

  //Set stepper to start position (how?)
  //Set potentiometer to start position (how?)

  //Set "Start" to 1
}

void loop() {
  // put your main code here, to run repeatedly:
 /* Start stepper motor.
  * Record how many steps until photo-resistors detects
  *  max.
  * Stop stepper motor (unless we want to constantly
  *  update it to the brightest light?)
  * Turn DC motor until, variable resistor is at the 
  *  same "degree" as stepper motor.
  * (Photocell should be connected to DC motor.)
  */

  //If "Start" = 1, ...
  
  //Let d be the distance moved.
  d = 0;
  //Set initial light value to 0
  
  //if loop to keep stepper running until max reading
  
    //This turns the stepper motor one step
    stepper->step(100, FORWARD, SINGLE);
            //Can also do backwards, interleave, microstep etc.
    //Record: d = d+1
    //Read the light value
    //Compare light value to previous readings.
    //If greater, record d.
    //If lesser, move forward one step?
    //Repeat until largest value is found.

   //When largest value is found,
   //TURN OFF STEPPER
   //Turn on motor

   //When potientiometer value = d
   //Turn off motor
   //The Photocell should now be facing the maximum light.

   //Set "Start" to 0
}
