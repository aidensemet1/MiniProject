#include <Encoder.h>

Encoder Lwheel(2,11); //encoder pins on the Arduino for the left motor wheel
Encoder Rwheel(3,5);  //encoder pins on the Arduino for the right motor wheel


/*----------------------------------------------------------
 * SETUP PINS (don't change these, they are specified on the 
 * motor driver data sheet)
 *----------------------------------------------------------
 */

int D2 = 4; //motor board enable pin
int mLDirPin = 7; //Left motor direction pin
int mLSpeedPin = 9; //Left motor speed pin 
int mRDirPin = 8; //right motor direction pin
int mRSpeedPin = 10; //right motor speed pin

/*----------------------------------------------------------
 * Constants
 *----------------------------------------------------------
 */

float radPerCount = 0.001963;
int period  = 5;
float d = 12.0;
float radius  = 6.0;
float phiDotDesired = 0;
//float rhoDotDesired = 15;


void setup() {
  Serial.begin(250000);   //starts serial communication w/ baud rate 250000
  motorSetup();   //motorSetup() function call that is defined below
}

/*------------------------------------------------------------
 * Initialized motor angular position/velocity variables and
 *------------------------------------------------------------
 */

float leftTheta = 0;
float rightTheta = 0;
float CRho = 0;
float CPhi = 0;

/*------------------------------------------------------
 * Initialized motor control VELOCITY variables
 *------------------------------------------------------
 */

float KiRhoDot = 2;
float KiPhiDot = 2;

float KdRhoDot = 20;
float KdPhiDot = 2;

float KpRhoDot = 7;
float KpPhiDot = 7;

float IRhoDot = 0;
float IPhiDot = 0;

float phiDot;
float rhoDot;
float eRhoDot;
float ePhiDot;
float DPhi;
float CPhiDot;
float CRhoDot;
float eRhoDotPast = 0;
float ePhiDotPast = 0;

/*------------------------------------------------------
 * Initialized motor control POSITION variables
 *------------------------------------------------------
 */

float Ki = 0;
float Kd = 0;
float Kp = 5;

float rhoDesired = 120.0;
float eRhoPast = 0;
float IRho = 0;
float numRotations = 0;
float desiredDist = 120; // in inches
float desiredRotations = (desiredDist * 4) / (2 * 6.28 * radius);

void loop() {
  while (numRotations <= desiredRotations) {
    unsigned long startTime = millis();
  
    //Position variables
    float newLeftTheta = getLeftTheta();
    float newRightTheta = getRightTheta();
  
    //get current position (rho)
    float currentRho = radius * 0.5 * (newLeftTheta + newRightTheta);

    numRotations = currentRho / (6.28 * radius);
    //Serial.println(currentRho / (6.28 * radius));
    
    float eRho = rhoDesired - currentRho;
    //Serial.println(currentRhoe);
    float DRho = (eRho - eRhoPast) / period;
    float IRho = IRho + eRho * period;
   
    float cRho = eRho * Kp + Kd * DRho + Ki * IRho; // this will give the desired rhoDot
    
    //Velocity variables
    float wLeft = 1000 * (newLeftTheta - leftTheta) / period;
    float wRight = 1000 * (newRightTheta - rightTheta) / period;
  
    //get dot controller variables and write motor speed
    dotController(newLeftTheta, newRightTheta, wLeft,wRight, 15);
    writeToMotor(CRhoDot, CPhiDot);
    
    //--------------------------------------------------
    // set past values
    //--------------------------------------------------
    
    eRhoDotPast = eRhoDot;
    ePhiDotPast = ePhiDot;
    eRhoPast = eRho;
  
    leftTheta = newLeftTheta;
    rightTheta = newRightTheta;
   
    while(millis() < startTime + period){}
    }
    motorStop();
    delay(5000);
 }

//----------------------------------------------------------
// Calculate controller for PhiDot and RhoDot and set the
// global variables
//----------------------------------------------------------
void dotController(float newLeftTheta, float newRightTheta, float wLeft, float wRight, float rhoDotDesired) {
  float phiDot = ((radius /d) * (wLeft - wRight));
  float rhoDot = (radius * 0.5 *(wLeft + wRight));

  eRhoDot =rhoDot - rhoDotDesired;
  ePhiDot = phiDot - phiDotDesired;

  float DRhoDot = (eRhoDot - eRhoDotPast) / period;
  float DPhiDot = (ePhiDot - ePhiDotPast) / period;

  IRhoDot = IRhoDot + eRhoDot * period;
  IPhiDot = IPhiDot + ePhiDot * period;

  CRhoDot = eRhoDot * KpRhoDot + KdRhoDot * DRhoDot + KiRhoDot * IRhoDot; // this will give the sum Vbar
  CPhiDot = ePhiDot * KpPhiDot + KdPhiDot * DPhiDot + KiPhiDot * IPhiDot; // this will give the diffence delta V
}

//-------------------------------------------------------------
// Calculate motor voltages and write to motor based off of
// contoller values
//-------------------------------------------------------------
void writeToMotor(float CRho, float CPhi) {
  float Vr = (0.5 * (CRho + CPhi));
  float Vl = (0.5 * (CRho - CPhi));

  if (Vr >= 0 ) {
    digitalWrite(mRDirPin, LOW);
    analogWrite(mRSpeedPin, Vr);
  } else {
    digitalWrite(mRDirPin, HIGH);
    analogWrite(mRSpeedPin, -1*Vr);
  }

  if (Vl >= 0 ) {
    digitalWrite(mLDirPin, LOW);
    analogWrite(mLSpeedPin, Vr);
  } else {
    digitalWrite(mLDirPin, HIGH);
    analogWrite(mLSpeedPin, -1*Vr);
  }
}

/*------------------------------------------------------------
 * helper funcition to get angular position of right wheel
 *------------------------------------------------------------
 */
double getRightTheta() {
  long rightCount = Rwheel.read();
  return (rightCount * radPerCount);
}

/*----------------------------------------------------------
 * helper funciton to get angular position of left wheel
 *----------------------------------------------------------
 */
double getLeftTheta() {
  long leftCount = Lwheel.read();
  return (leftCount * radPerCount);
}

/*----------------------------------------------------------
 * helper funciton to setup both motors
 *----------------------------------------------------------
 */
void motorSetup() {
  pinMode(mRDirPin, OUTPUT);
  pinMode(mRSpeedPin, OUTPUT);
  pinMode(mLDirPin, OUTPUT);
  pinMode(mLSpeedPin, OUTPUT);
  pinMode(D2, OUTPUT);
  digitalWrite(D2,HIGH); //enable motor driver board
  digitalWrite(mRDirPin, LOW);
  digitalWrite(mLDirPin, LOW);
  analogWrite(mRSpeedPin, 0); //set motor voltage to 0
  analogWrite(mLSpeedPin, 0); //set motor voltage to 0
}
/*------------------------------------------------------------
 * helper function that stops the motor wheels from spinning
 *------------------------------------------------------------
*/
void motorStop() {
  analogWrite(mRSpeedPin, 0);
  analogWrite(mLSpeedPin, 0);
}
