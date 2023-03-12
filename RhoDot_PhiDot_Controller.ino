#include <Encoder.h>

Encoder Lwheel(2,11);
Encoder Rwheel(3,5);

float deltaV = 0;
float barV = 1;

//SETUP PINS (don't change these, they are specified on the motor driver data sheet)
int D2 = 4; //motor board enable pin
int mLDirPin = 7; //Left motor direction pin
int mLSpeedPin = 9; //Left motor speed pin 
int mRDirPin = 8; //right motor direction pin
int mRSpeedPin = 10; //right motor speed pin


// Constants
float radPerCount = 0.001963;
int period  = 10;
float d = 12.0;
float radius  = 6.0;
float phiDotDesired =0;
float rhoDotDesired =0;
float eRhoDotPast =0;
float ePhiDotPast =0;

void setup() {
  // put your setup code here, to run once:

}
float leftTheta =0;
float rightTheta = 0;


void loop() {
  unsigned long startTime = millis();
  
  float newLeftTheta = getLeftTheta();
  float newRightTheta = getRightTheta();
  
  float dLeft = (newLeftTheta - leftTheta) / period;
  float dRight = (newRightTheta - rightTheta) / period;
  float phiDot = (radius /d) * (dLeft - dRight);
  float rhoDot = radius * 0.5 *(dLeft + dRight);
  
  float eRhoDot =rhoDot - rhoDotDesired;
  float ePhiDot = phiDot - phiDotDesired;
  float deltaERhoDot = eRhoDot - eRhoDotPast;
  float deltaEPhiDot = ePhiDot - ePhiDotPast;
  


  eRhoDotPast = eRhoDot;
  ePhiDotPast = ePhiDot;
  while(millis() < startTime + period){}
}



double getRightTheta() {
  long rightCount = Rwheel.read();
  return (rightCount * radPerCount);
}

double getLeftTheta() {
  long leftCount = Lwheel.read();
  return (leftCount * radPerCount);
}


void motorSetup() {
  pinMode(mRDirPin, OUTPUT);
  pinMode(mRSpeedPin, OUTPUT);
  pinMode(mLDirPin, OUTPUT);
  pinMode(mLSpeedPin, OUTPUT);
  pinMode(D2, OUTPUT);
  digitalWrite(D2,HIGH); //enable motor driver board
  analogWrite(mRSpeedPin, 0); //set motor voltage to 0
  analogWrite(mLSpeedPin, 0); //set motor voltage to 0
}
