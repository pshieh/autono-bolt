#include <Wire.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include "VL6180X.h"

VL6180X sensor;

#define FULL4WIRE 4

// Motor pin definitions
#define motorPin1  6     // A1 
#define motorPin2  7     // A2 
#define motorPin3  8     // B1 
#define motorPin4  9     // B2 

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
//AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

AccelStepper stepperL (FULL4WIRE, 6, 7, 8, 9, true);
AccelStepper stepperR (FULL4WIRE, 10, 11, 12, 13, true);

int x_steps = 1600;
int y_steps = 300;
int steps = 0;
int locationSteps = 150;
int address = 0;

volatile int flag = 0;
int dataflag = 0;

int  totalIntSteps = 0;
int  totalsteps = 0;

void setup(){
  Serial.begin(9600);

  Wire.begin();
  ///////////////////////////////////LIDAR SENSOR///////////////////////////////////////
  sensor.ParameterTransfer(); //removes flag and signals that data is ready to transfer
  sensor.configureDefault();
  sensor.setTimeout(500);
  sensor.ClearAllInterrupt(); //clears interrupt so that interrupt can be triggered again

  sensor.ParameterHoldInit(); //sets flag that data is not stable and not ready to transfer
  sensor.RangeSetInterMeasPeriod(); //sets time delay in between range measurements
  sensor.setupGPI01(); //sets GPIO1 as an interrupt output
  //sensor.ClearAllInterrupt(); //clears interrupt so that interrupt can be triggered again
  
  sensor.RangeConfigInterrupt(); //sets low threshold
  sensor.RangeSetThreshold(); //sets range
  sensor.ParameterTransfer(); //removes flag and signals that data is ready to transfer
    
  sensor.startRangeContinuous(); //begins continuous mode
  
  ///////////////////////////////////INTERRUPT///////////////////////////////////////
  const byte interruptPin1 = 2;
  //const byte interruptPin2 = 3;

  pinMode(interruptPin1, INPUT);
  //pinMode(interruptPin2, INPUT);
  
  ///////////////////////////////////ACCELSTEPPER///////////////////////////////////////
  const int MaxSpeed = 100; 
  const int Acceleration = 40;
  const int Speed = 100;
  
  stepperL.setMaxSpeed(MaxSpeed);
  stepperL.setAcceleration(Acceleration);
  stepperL.setSpeed(Speed);
  
  stepperR.setMaxSpeed(MaxSpeed);
  stepperR.setAcceleration(Acceleration);
  stepperR.setSpeed(Speed);

  
  ///////////////////////////////////INTERRUPT///////////////////////////////////////
  attachInterrupt(digitalPinToInterrupt(interruptPin1), align, FALLING);

  ///////////////////////////////////CLEAR EEPROM////////////////////////////////////
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void loop(){
  if (dataflag==0){
  totalsteps = EEPROM.read(address);
  address = address + 1;
  if (address == EEPROM.length()) {
    address = 0;
    dataflag = 1;
  }
}
  
  ///////////////////////////////////ALIGN AND TIGHTEN///////////////////////////////////////
 if (flag == 1){
    while (totalIntSteps < locationSteps && flag ==1){
      Serial.print("IntDownSteps: "); Serial.println(totalIntSteps);
      stepperL.moveTo(y_steps);
      stepperR.moveTo(y_steps);
      stepperL.run();
      stepperR.run();
      totalIntSteps = totalIntSteps + 1;
    }
    totalIntSteps = 0;
  //////////////////////////////////LOWER AND SCREW//////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////
   while (totalIntSteps < locationSteps && flag ==1){
      Serial.print("IntUpSteps: "); Serial.println(totalIntSteps);
      stepperL.moveTo(-y_steps);
      stepperR.moveTo(-y_steps);
      stepperL.run();
      stepperR.run();
      totalIntSteps = totalIntSteps + 1;
    }
    flag = 0;
    totalIntSteps = 0;
    sensor.ClearAllInterrupt();
  }
  ///////////////////////////////////SWEEP FOR BOLTS///////////////////////////////////////
 if(flag == 0){
    while (stepperL.currentPosition() < x_steps && flag ==0){ //left function
      Serial.print("TotxSteps: "); Serial.println(totalsteps);
      stepperL.moveTo(x_steps);
      stepperR.moveTo(-x_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
          if (dataflag==0){
      EEPROM.update(address, totalsteps);
      address = address + 1;
        if (address == EEPROM.length()) {
          address = 0;
          dataflag = 1;
        }
        dataflag=0;
    }
    }
  sensor.ClearAllInterrupt();
  totalsteps = 0;
  while (totalsteps < y_steps && flag ==0){ //down function
      Serial.print("TotySteps: "); Serial.println(totalsteps);
      stepperL.moveTo(y_steps);
      stepperR.moveTo(y_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
    }
  totalsteps = 0;
  sensor.ClearAllInterrupt();
    
  while (totalsteps < x_steps && flag ==0){ //right function
      stepperL.moveTo(-x_steps);
      stepperR.moveTo(x_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
  }
  totalsteps = 0;
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  sensor.ClearAllInterrupt();
  
  while (totalsteps < y_steps && flag ==0){ //down function
      stepperL.moveTo(y_steps);
      stepperR.moveTo(y_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
  }
  totalsteps = 0;
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  sensor.ClearAllInterrupt();
    
  while (totalsteps < x_steps && flag ==0){ //left function
      stepperL.moveTo(x_steps);
      stepperR.moveTo(-x_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
  }
  totalsteps = 0;
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  sensor.ClearAllInterrupt();
  }
}

void align(){
  flag = 1;
  //sensor.ClearAllInterrupt();
  return;
}
///////////////////////////////////MOVEMENT FUNCTIONS////////////////////////////////
/*
void up(){
  while (stepperL.currentPosition() < y_steps){
    stepperL.moveTo(-y_steps);
    stepperR.moveTo(-y_steps);
    stepperL.run();
    stepperR.run();
  }
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  return;
}

void down(){
  while (stepperL.currentPosition() < y_steps){
    stepperL.moveTo(y_steps);
    stepperR.moveTo(y_steps);
    stepperL.run();
    stepperR.run();
  }
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  return;
}

void left(){
  while (stepperL.currentPosition() < x_steps){
    stepperL.moveTo(x_steps);
    stepperR.moveTo(-x_steps);
    stepperL.run();
    stepperR.run();
  }
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  return;
}

void right(){
  while (stepperL.currentPosition() < x_steps){
    stepperL.moveTo(-x_steps);
    stepperR.moveTo(x_steps);
    stepperL.run();
    stepperR.run();
  }
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  return;
}
*/
