#include <Encoder.h>

Encoder Lwheel(3,5);
Encoder Rwheel(2,11);


/*----------------------------------------------------------
 *SETUP PINS (don't change these, they are specified on the 
 *motor driver data sheet)
 *----------------------------------------------------------
*/
int D2 = 4; //motor board enable pin
int mLDirPin = 7; //Left motor direction pin
int mLSpeedPin = 9; //Left motor speed pin 
int mRDirPin = 8; //right motor direction pin
int mRSpeedPin = 10; //right motor speed pin


/*----------------------------------------------------------
 * Constants
 * ----------------------------------------------------------
 */

float radPerCount = 0.001963;
int period  = 5;
float d = 12.0;
float radius  = 6.0;
float phiDotDesired =0;
float rhoDotDesired =15;
float eRhoDotPast =0;
float ePhiDotPast =0;


void setup() {
  Serial.begin(250000);
  motorSetup();

}

/*----------------------------------------------------------
 * Initialized motor angular position/velocity variables and
 * ----------------------------------------------------------
 */

float leftTheta =0;
float rightTheta = 0;
float CRho =0;
float CPhi = 0;

/*----------------------------------------------------------
 * Initialized motor control VELOCITY variables
 * ----------------------------------------------------------
 */

float KiRho = 2;
float KiPhi =2;

float KdRho =20;
float KdPhi =2;

float KpRho =7;
float KpPhi = 7;



float IRho =0;
float IPhi =0;


/*----------------------------------------------------------
 * Initialized motor control POSITION variables
 * ----------------------------------------------------------
 */





void loop() {
  
  unsigned long startTime = millis();

  //----------------------------------------------------------
  // Get all the needed controller values
  //----------------------------------------------------------
  
  //Position variables
  float newLeftTheta = getLeftTheta();
  float newRightTheta = getRightTheta();



  //Velocity variables
  float wLeft = 1000 * (newLeftTheta - leftTheta) / period;
  float wRight = 1000 * (newRightTheta - rightTheta) / period;
  
  float phiDot = ((radius /d) * (wLeft - wRight));
  float rhoDot = (radius * 0.5 *(wLeft + wRight));

  float eRhoDot =rhoDot - rhoDotDesired;
  float ePhiDot = phiDot - phiDotDesired;

  //----------------------------------------------------------
  // Calculate controller outputs
  //----------------------------------------------------------
  
  float DRho = (eRhoDot - eRhoDotPast) / period;
  float DPhi = (ePhiDot - ePhiDotPast) / period;

  IRho = IRho + eRhoDot * period;
  IPhi = IPhi + ePhiDot * period;

  float CRho = eRhoDot * KpRho + KdRho * DRho + KiRho * IRho; // this will give the sum Vbar
  float CPhi = ePhiDot * KpPhi + KdPhi * DPhi + KiPhi * IPhi; // this will give the diffence delta V


  writeToMotor(CRho, CPhi);
  
  //----------------------------------------------------------
  // set past error values
  //----------------------------------------------------------
  
  eRhoDotPast = eRhoDot;
  ePhiDotPast = ePhiDot;

  leftTheta = newLeftTheta;
  rightTheta = newRightTheta;
  
 
  while(millis() < startTime + period){}
}



//----------------------------------------------------------
// Calculate motor voltages and write to motor based off of
// contoller values
//----------------------------------------------------------
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


/*----------------------------------------------------------
 * helper funcition to get angular position of right wheel
 * ----------------------------------------------------------
 */
double getRightTheta() {
  long rightCount = Rwheel.read();
  return (rightCount * radPerCount);
}

/*----------------------------------------------------------
 * helper funciton to get angular position of left wheel
 * ----------------------------------------------------------
 */
double getLeftTheta() {
  long leftCount = Lwheel.read();
  return (leftCount * radPerCount);
}


/*----------------------------------------------------------
 * helper funciton to setup both motors
 * ----------------------------------------------------------
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
