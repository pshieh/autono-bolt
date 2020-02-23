#include <Wire.h>
#include <AccelStepper.h>
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

volatile int flag = 0;
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
  const byte interruptPin1 = 2;
  const byte interruptPin2 = 3;
  const byte interruptPin3 = 18;
  
  pinMode(interruptPin1, INPUT);
  pinMode(interruptPin2, INPUT);
  pinMode(interruptPin3, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin1), align, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), homingx, FALLING); //x-axis bump sensor
  attachInterrupt(digitalPinToInterrupt(interruptPin3), homingy, FALLING); //y-axis bump sensor
}

void loop(){
  ///////////////////////////////////HOME X///////////////////////////////////////
 if (flag == 0){ //stepping left
    while (totalsteps < locationSteps && flag ==0){
      Serial.print("HomingXSteps: "); Serial.println(totalIntSteps);
      stepperL.moveTo(x_steps);
      stepperR.moveTo(-x_steps);
      stepperL.run();
      stepperR.run();
      totalIntSteps = totalIntSteps + 1;
    }
  } 
 ////////////////////////////////////HOME Y///////////////////////////////////////
 if (flag = 1){ //stepping down
  stepperL.stop();
  stepperR.stop();
  totalsteps = 0;
  
   while (totalsteps < locationSteps && flag ==1){ 
      Serial.print("HomingYSteps: "); Serial.println(totalIntSteps);
      stepperL.moveTo(y_steps);
      stepperR.moveTo(y_steps);
      stepperL.run();
      stepperR.run();
      totalIntSteps = totalIntSteps + 1;
    }
  }
 /////////////////////////////////STARTING POSITION//////////////////////////////////
 if (flag == 2){
  stepperL.stop();
  stepperR.stop();
  totalsteps = 0;
  
    //move to starting position
    while (totalIntSteps < locationSteps && flag ==2){
      Serial.print("IntDownSteps: "); Serial.println(totalIntSteps);
      stepperL.moveTo(y_steps);
      stepperL.run();
      stepperR.run();
      totalIntSteps = totalIntSteps + 1;
    }
    totalIntSteps = 0;
    totalsteps = 0;
    flag = 3;
  }
  ///////////////////////////////////SWEEP FOR BOLTS///////////////////////////////////////
 if(flag == 3){
    while (stepperL.currentPosition() < x_steps && flag ==3){ //right function
      Serial.print("TotxSteps: "); Serial.println(totalsteps);
      stepperL.moveTo(-x_steps);
      stepperR.moveTo(x_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
    }
  sensor.ClearAllInterrupt();
  totalsteps = 0;
  while (totalsteps < y_steps && flag ==3){ //up function
      Serial.print("TotySteps: "); Serial.println(totalsteps);
      stepperL.moveTo(-y_steps);
      stepperR.moveTo(-y_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
    }
  totalsteps = 0;
  sensor.ClearAllInterrupt();
    
  while (totalsteps < x_steps && flag ==3){ //left function
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
  
  while (totalsteps < y_steps && flag ==3){ //up function
      stepperL.moveTo(-y_steps);
      stepperR.moveTo(-y_steps);
      totalsteps = totalsteps + 1;
      stepperL.run();
      stepperR.run();
  }
  totalsteps = 0;
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  sensor.ClearAllInterrupt();
    
  while (totalsteps < x_steps && flag ==3){ //right function
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
  }


////////////////////////////////////BOLT SCREW MODE//////////////////////////////////////////
if (flag = 4)
   while (totalIntSteps < locationSteps && flag ==4){
      Serial.print("IntUpSteps: "); Serial.println(totalIntSteps);
      stepperL.moveTo(y_steps);
      stepperR.moveTo(y_steps);
      stepperL.run();
      stepperR.run();
      totalIntSteps = totalIntSteps + 1;
    }
  ///////////////////////////////////////////LOWER AND SCREW////////////////////////////////////////////////
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
   while (totalIntSteps < locationSteps && flag ==4){
      Serial.print("IntUpSteps: "); Serial.println(totalIntSteps);
      stepperL.moveTo(-y_steps);
      stepperR.moveTo(-y_steps);
      stepperL.run();
      stepperR.run();
      totalIntSteps = totalIntSteps + 1;
    }
    flag = 3; //sends to bolt sweep mode
    totalIntSteps = 0;
    sensor.ClearAllInterrupt();
}

void homingx(){
  flag = 1; //sends to motor stop mode
  //sensor.ClearAllInterrupt();
  return;
}
void homingy(){
  flag = 2; //sends to motor stop mode
  //sensor.ClearAllInterrupt();
  return;
}
void align(){
  flag = 4; //sends to bolt screw mode
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
