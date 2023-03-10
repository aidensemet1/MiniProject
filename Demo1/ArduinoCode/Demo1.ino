unsigned long Tc; //current time =
int N = 1; // loop iterations
#include <Encoder.h>   // encoder Arduino library

//distance and angle
float desiredDistance = 2; //in feet
float desiredAngle = 0; //in degrees

//encoder pins for localization (2 and 3 are interrupt pins)
Encoder Lwheel(2, 4);
Encoder Rwheel(3, 5);

String read_setpoint;
float ref;  //radians for the wheel to turn
float theta = 0;
long count = 0;
long newCountR;
long newCountL;

//constants
float radPerCount = 0.001963;
float pi = 3.14159;
float diameter = 6; //inches
float vehicleRadius = 6; //distance from centerpoint of vehicle to the wheel (inches)

float radius = diameter/2; //radius of the wheels (inches)
float circumference = pi * diameter; //circumference of the wheels (inches)
float requiredRadiansFwd; //radians required for vehicle to move forward
float requiredRadiansTrn; //radians required for vehicle to turn in place
float desiredAngleRad = (pi/180)*desiredAngle; //desired angle converted to radians
float rotationDistance = vehicleRadius*desiredAngleRad; //distance for each wheel to travel to achieve desired angle
bool hasTurned = false;

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

int C;
int Ctrn;

 void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);  //start serial communication w/ baud rate of 250000
  motorSetup();
  
  //distance calculations
  requiredRadiansFwd = ((desiredDistance*12)/(circumference/2))*pi; //radians required to travel desired distance
  
  //turn in place calculations
  requiredRadiansTrn = ((rotationDistance)/(circumference/2))*pi; //radians required to turn in place to desired angle
  
  if (desiredAngle == 0) {
    hasTurned = true;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  //turning

      Tc = millis(); //get current time ms
      e = requiredRadiansTrn - getCurrentPos();
      Ie = Ie + e * Ts*0.001; // calculating the integral
      Ctrn = Kp* e + Ie* Ki; // This will be in volts
      C = Kp* e + Ie* Ki; // This will be in volts
 
      if (desiredAngle >= 0) { //clockwise
        //left motor
        digitalWrite(mLDirPin, LOW);
        analogWrite(mLSpeedPin, int(51*Ctrn));
        //right motor
        digitalWrite(mRDirPin, HIGH);
        analogWrite(mRSpeedPin, int(51*Ctrn));
      } else { //counterclockwise
        //left motor
        digitalWrite(mLDirPin, HIGH);
        analogWrite(mLSpeedPin, int(51*C));
        //right motor
        digitalWrite(mRDirPin, LOW);
        analogWrite(mRSpeedPin, int(51*C));
      }

  if (e == 0) {
    hasTurned = true;
  }

  if (hasTurned == true) {
   //forward
   Tc = millis(); //get current time ms
   e = requiredRadiansFwd - getCurrentPos();
   Ie = Ie + e * Ts*0.001; // calculating the integral
   C = Kp* e + Ie* Ki; // This will be in volts
   if (C >=0) { //forward ?
     digitalWrite(mLDirPin, LOW);
     analogWrite(mLSpeedPin, int(51*C));
     digitalWrite(mRDirPin, LOW);
     analogWrite(mRSpeedPin, int(51*C));
   } else { //backward ?
     digitalWrite(mLDirPin, HIGH);
     analogWrite(mLSpeedPin, int(-51*C));
     digitalWrite(mRDirPin, HIGH);
     analogWrite(mRSpeedPin, int(-51*C));
   }
  }
  

  while(millis() < Tc + Ts){}
}


//function that gets the current position of the Right wheel
float getRCurrentPos() {
  newCountR = Rwheel.read();
  thetaR = newCountR * radPerCountR;
  return thetaR;
}

//function that gets the current position of the left wheel
float getLCurrentPos() {
 newCountL = Lwheel.read();
 thetaL = newCountL * radPerCountL;
 return thetaL;
}



// This function initilizes the motor, direction, and enable pins
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
