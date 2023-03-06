unsigned long Tc; //current time =
int N = 1; // loop iterations
#include <Encoder.h>   // encoder Arduino library

//distance and angle
float desiredDistance = 2; //in feet
float requiredRadians; //calculated radians required to turn based on desired distance to travel
float desiredAngle = 90; //in degrees

//encoder pins for localization (2 and 3 are interrupt pins)
Encoder Lwheel(2, 4);
Encoder Rwheel(3, 5);

String read_setpoint;
float ref;  //radians for the wheel to turn
float theta = 0;
long count = 0;
long newCount;
float radPerCount = 0.001963;
float pi = 3.14159;
float diameter = 5.5;
float radius = diameter/2;
float circumference = pi * diameter;

//SETUP PINS (don't change these, they are specified on the motor driver data sheet)
int D2 = 4; //motor board enable pin
int mLDirPin = 7; //Left motor direction pin
int mLSpeedPin = 9; //Left motor speed pin 
int mRDirPin = 8; //right motor direction pin
int mRSpeedPin = 10; //right motor speed pin


// Initializing controller gains
float Kp = 10; // Proportional gain
float Ki = 0; // Integral gain

// error variables
float Ie; // running integral of error
float e; // current error

// Time variables
float Ts = 5; // sampling rate I think?(10 ms)

 void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);  //start serial communication w/ baud rate of 250000
  motorSetup();
  
  //distance calculations
  requiredRadians = ((desiredDistance*12)/(circumference/2))*pi; //radians required to travel desired distance
}

void loop() {
  // put your main code here, to run repeatedly:

  Tc = millis(); //get current time ms
  e = requiredRadians - getCurrentPos();
  Ie = Ie + e * Ts*0.001; // calculating the integral
  int C = Kp* e + Ie* Ki; // This will be in volts

  if (C >=0) {
    digitalWrite(mLDirPin, LOW);
    analogWrite(mLSpeedPin, int(51*C));
  } else {
    digitalWrite(mLDirPin, HIGH);
    analogWrite(mLSpeedPin, int(-51*C));
  }

  while(millis() < Tc + Ts){}
}

/*
//function that gets the current position of the wheel
//used in our testing & tuning of the motor
float getCurrentPos() {
  newCount = wheel.read();
  theta = newCount * radPerCount;
  return theta;
}
*/

// This function initilizes the motor, direction, and enable pins
void motorSetup() {
  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);
  pinMode(D2, OUTPUT);
  digitalWrite(D2,HIGH); //enable motor driver board
  analogWrite(m1SpeedPin, 0); //set motor voltage to 0
}
