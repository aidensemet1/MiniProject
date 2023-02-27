
unsigned long Tc; //current time =
int N = 1; // loop iterations
#include <Encoder.h>   // encoder Arduino library

Encoder wheel(2, 3);   //pins on Arduino for motor
String read_setpoint;
float ref;  //reference position for wheel position
float theta = 0;
long count = 0;
long newCount;
float radPerCount = 0.001963;

// Setup pins
int m1DirPin = 7; //motor direction pin
int m1SpeedPin = 9; //motor speed pin
int D2 = 4; //motor board enable pin

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
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    read_setpoint = Serial.readString();  //get data sent from Pi
    //Serial.println(read_setpoint);
    if (read_setpoint == "0") {  //if received setpoint = 0
      ref = 0;                   //set ref position = 0
      Serial.print(ref);         //print to serial monitor
    }
    else if (read_setpoint == "pi/2") {   //if received setpoint = pi/2
      ref = 1.57;                         //set ref position = pi/2 = 1.57
      Serial.print(ref);                  //print to serial monitor
    }
    else if (read_setpoint == "pi") {    //if received setpoint = pi
      ref = 3.14;                        //set ref position = pi = 3.14
      Serial.print(ref);                 //print to serial monitor
    }
    else if (read_setpoint == "3pi/2") {   //if received setpoint = 3pi/2
      ref = 4.71;                          //set red position = 3pi/2 = 4.71
      Serial.print(ref);                   //print to serial monitor
    }
  }
  
  Tc = millis(); //get current time ms
  e = ref - getCurrentPos();
  Ie = Ie + e * Ts*0.001; // calculating the integral
  int C = Kp* e + Ie* Ki; // This will be in volts

  if (C >=0) {
    digitalWrite(m1DirPin, LOW);
    analogWrite(m1SpeedPin, int(51*C));
  } else {
    digitalWrite(m1DirPin, HIGH);
    analogWrite(m1SpeedPin, int(-51*C));
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
