#include <Encoder.h>

Encoder Lwheel(2,11);
Encoder Rwheel(3,5);

int D2 = 4;
int mLDirPin = 7;
int mLSpeedPin = 9;
int mRDirPin = 8;
int mRSpeedPin = 10;

int desired_Distance = 10;
int counts_Per_Rev = 3200;
float diameter = 6;
float circum = PI * diameter;
      
void setup() {
  Serial.begin (9600);
  motorSetup();
}

void loop() {
  // put your main code here, to run repeatedly
  if (desired_Distance == 10) {
    startcar();
    delay(7950);   
    stopcar();
    exit(0);
  } else {
    exit(0);
  }
}

void startcar() {
  digitalWrite(mRDirPin, HIGH);
  digitalWrite(mLDirPin, HIGH);
  analogWrite(mRSpeedPin, 122);
  analogWrite(mLSpeedPin, 104.5);
}

void stopcar(){
  digitalWrite(mRDirPin, LOW);
  digitalWrite(mLDirPin, LOW);
  analogWrite(mRSpeedPin, 0);
  analogWrite(mLSpeedPin, 0);
}

void motorSetup () {
  pinMode(mRDirPin, OUTPUT);
  pinMode(mRSpeedPin, OUTPUT);
  pinMode(mLDirPin, OUTPUT);
  pinMode(mLSpeedPin, OUTPUT);
  pinMode(D2, OUTPUT);
  digitalWrite(D2, HIGH);
  analogWrite(mRSpeedPin, 0);
  analogWrite(mLSpeedPin, 0);
}
