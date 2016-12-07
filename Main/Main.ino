/* ARDUINO PROJECT
 * Aim: Detect the direction of brightest light 
 * and turn a solar cell to face that direction. 
 * 
 * Date Created: 28 November 2016
 * Date Updated: 7 December 2016
 * 
 * Contributors
 *   Keven Baquero
 *   David Verdi
 *   Adeliaide Young
 *   
 * Notes
 *  The Solar panel is connected to the DC motor. 
 *  The DC motor is connected to the 10k, Multi-Turn potentiometer, via three gears.
 *  
 *  The Photoresistor is connected to the stepper motor.
 *  
 *  All acutation is calibrated. First, the stepper must be manually homed. 
 *  
 *  Operation:
 *  1. Stepper motor is homed to a marked zero position - DONE
 *  2. Arduino is powered on - DONE
 *  3. Solar panel is rotated to zero position (while loop to target voltage)
 *  3. Stepper motor rotates a full 360 degrees using however many steps. 
 *     For each step, the analog voltage is recorded.
 *     The voltage out will be maximum at the brightest spot. 
 *     The step that produced the maximum voltage will be saved
 *  4. A formula will convert the target step value to degrees
 *  5. A formula will convert the degrees to a target voltage for the potentiometer
 *  6. The the DC motor will actuate forward untill it hits the target voltage
 *  
 *  Experimental TODO:
 *  - Experimentally determine the number of steps per revolution
 *  - Experimentally determine the potentiometer zero voltage range
 *  - Experimentally determine the potentiometer 360 voltage range
 */



/*** Include Libraries ***/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


/*** Declare Analog Read pin Settings ***/
int pot_pin = A0;         //This is the potentiometer, which is connected to the DC motor/Solar Panel
int photoresist_pin = A1;   //This is the output from the photoresistor


/*** Declare Motor parameters ***/
int steps_per_rev = 200;   // Determine Experimentally
int stepper_port = 2;      // Port 2, which is M3 and M4
int dc_port = 2;           // DC motor is connected to M2
int dc_speed = 250;        // 0-255
int stepper_speed = 10;    // 10 rpm


/*** Declare Measured Voltage Parameters ***/
int dc_zero_min = 10;   // Experimentally Determined
int dc_zero_max = 15;
int dc_360_voltage = 500;   // Experimentally Determined

/*** Helper Function Parameters ***/
int readnumber = 5;


/*** Initialize motors ***/ 
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield ()
//Connect a stepper motor with 200 steps per revolution (1.8 degree) to motor port #2 (M3 and M4)
Adafruit_StepperMotor *stepper = AFMS.getStepper(steps_per_rev, stepper_port);
//Connect the DC motor to port M2
Adafruit_DCMotor *DCMotor = AFMS.getMotor(dc_port)


void setup() {
  Serial.begin(9600);        //set up Serial library at 9600 bps
  Serial.println("Light Aiming Beam (L.A.B.)\n");

  AFMS.begin();        // create with the default frequency 1.6kHz


  // Set Motor Speeds
  DCMotor->setSpeed(dc_speed);
  Serial.println("DC Motor speed set to:");
  Serial.println(dc_speed);
  Serial.println("\n");
  
  stepper->setSpeed(stepper_speed);
  Serial.println("Stepper Motor speed set to:");
  Serial.println(stepper_speed);
  Serial.println("\n");


  // Delay and Prompt user to zero Stepper Motor. 
  Serial.println("Please ensure that the stepper motor is in its zero position... \n");
  delay(5000) // 5 Second delay

  // Home DC Motor/Solar Panel
  position = read_sensor(int pot_pin);
  
  

  

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

/*** Helper Functions ***/
int read_sensor(int pin) {
  // Reads from the sensor five times and output average reading
  int reads;
  int sum = 0;

  for (reads = 0; reads < readnumber; reads++) {
    sum = sum + analog_read(pin);
  }
  int average = (sum / reads);
  return average;
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
}
