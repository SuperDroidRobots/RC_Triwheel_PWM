/*********************************************************

Tri-Wheel Vectoring Robot with RC Control
Brent Clay
SuperDroid Robots
December 1, 2015

This code uses an Arduino Uno mounted on a Tri-Wheel Vectoring platform
TP-093-003. The Arduino commands three PWM motor controllers (TE-058-000)
to control three independent omni-directional wheels. 

This firmware allows vectoring RC control of the robot's motion

Code is written for a Spektrum remote + receiver but could easily be reused
for other RC approaches

Platform:
http://www.superdroidrobots.com/shop/item.aspx/ig32-triangular-omni-wheel-vectoring-robot-platform/1458/

Motor Controller:
http://www.superdroidrobots.com/shop/item.aspx/pwm-motor-controller-3a-12-55v/583/

Vectoring Robot Support Page:
http://www.superdroidrobots.com/shop/custom.aspx/vectoring-robots/44/

Spektrum DX5e:
http://www.superdroidrobots.com/shop/item.aspx/spektrum-dx5etransmitter-with-ar610-receiver/992/

***********************************************************/

// Define pins for PWM motor controller
#define pwmA 6
#define dirA 7
//#define brkA 1

#define pwmB 5
#define dirB 4
//#define brkB 8

#define pwmC 3
#define dirC 2
//#define brkC 11

// Include necessary header files
#include "Arduino.h"

// Command struct for motor controllers
typedef struct {
	int pulse;
	bool direction;
	bool brake;
}	MotorValues;

// Defines structs for each motor
MotorValues motorA;
MotorValues motorB;
MotorValues motorC;

// *********************
// RC Vars
// *********************
unsigned long DRIVE_PULSE_WIDTH;
unsigned long TURN_PULSE_WIDTH;
unsigned long STRAFE_PULSE_WIDTH;
float pulseLow = 1051, pulseHigh = 1890;

// RC mappings -- strafe: aileron, drive: elevation, turn: rudder
int strafePinRC = 11, drivePinRC = 12, turnPinRC = 13;
float mFloat = 0, bFloat = 0;
float floatDeadband = 0.1;
float driveVal = 0, turnVal = 0, strafeVal = 0;

// Globals
float sideStep = 0.60; // Limiting factor to ensure direct side to side movement
float counterSteer = 1;
int rampLimit = 200;
int spdScaler = 1;


void setup() {
  
  // Uncomment the serial command to enable debugging through serial monitor
   Serial.begin(9600);		// initialize serial communication at 9600 bps

  // Set motor controller communication pins as outputs
  pinMode(dirA, OUTPUT);
  //pinMode(brkA, OUTPUT);
  pinMode(dirB, OUTPUT);
  //pinMode(brkB, OUTPUT);
  pinMode(dirC, OUTPUT);
  //pinMode(brkC, OUTPUT);
  
  // slope/intercept for converting RC signal to range [-1,1]
  mFloat = (float)2 / (pulseHigh - pulseLow);
  bFloat = -1*pulseLow*mFloat;
  
  // Command all motors to stop
  allStop();
 }

// Stops the Robot
void allStop() {
  motorA.pulse = 0; motorB.pulse = 0; motorC.pulse = 0;
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
  analogWrite(pwmC, 0);
}

// Convert RC signals to values from -1 to 1
float convertRCtoFloat(unsigned long pulseWidth) {
  // deadband
  if(pulseWidth > 1450 && pulseWidth < 1550) { pulseWidth = (float)(pulseHigh + pulseLow) / 2; }
  
  float checkVal = mFloat*pulseWidth + bFloat - 1;
  checkVal = checkVal < -1 ? -1 : checkVal;
  checkVal = checkVal >  1 ?  1 : checkVal;
  
  return checkVal;
}

// Scale pulse outputs
void normalizePulses()
{
  float a = motorA.pulse, b = motorB.pulse, c = motorC.pulse;
  
  float len = sqrt(a*a + b*b + c*c);
  if(len < 1) { return; }
  
  motorA.pulse = spdScaler*(float)rampLimit * a / len;
  motorB.pulse = spdScaler*(float)rampLimit * b / len;
  motorC.pulse = spdScaler*(float)rampLimit * c / len;
}

float getAbsolute(float val) {
  val = val < 0 ? -1*val : val;
  return val;  
}

float addDrive(float spd) {
  motorA.pulse += 0; motorB.pulse -= spd*rampLimit; motorC.pulse += spd*rampLimit;
}

float addStrafe(float spd) {
  motorA.pulse += 0; motorB.pulse -= spd*rampLimit; motorC.pulse -= spd*rampLimit;
}

float addTurn(float spd) {
  // countersteer to prevent rotation from strafing components
  motorA.pulse = (float)(-1*motorB.pulse - motorC.pulse)*counterSteer;
  // add in desired rotation based on input omega
  motorA.pulse += spd*rampLimit; motorB.pulse += spd*rampLimit; motorC.pulse += spd*rampLimit;  
}

float setDirection() {
  bool dir = LOW;
  dir = motorA.pulse > 0 ? HIGH : LOW; digitalWrite(dirA, dir);
  dir = motorB.pulse > 0 ? HIGH : LOW; digitalWrite(dirB, dir);
  dir = motorC.pulse > 0 ? HIGH : LOW; digitalWrite(dirC, dir);
  
  motorA.pulse = getAbsolute(motorA.pulse);
  motorB.pulse = getAbsolute(motorB.pulse);
  motorC.pulse = getAbsolute(motorC.pulse);
  
  if(motorA.pulse > spdScaler*rampLimit || motorB.pulse > spdScaler*rampLimit ||
    motorC.pulse > spdScaler*rampLimit)
  {
    normalizePulses();
  }
}

void normalizeVectors() {
  float a = driveVal, b = turnVal, c = strafeVal;
  float len = sqrt(a*a + b*b + c*c);
  if(len < 1) { return; }
  
  driveVal  = a / len;
  turnVal   = b / len;
  strafeVal = c / len;
}

void commandMotors() {
  	analogWrite(pwmA, motorA.pulse);
	analogWrite(pwmB, motorB.pulse);
	analogWrite(pwmC, motorC.pulse);
}

void loop() 
{
  // Read in the RC pulses
  DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH);//, PULSEIN_TIMEOUT);
  TURN_PULSE_WIDTH  = pulseIn(turnPinRC, HIGH);//, PULSEIN_TIMEOUT);
  STRAFE_PULSE_WIDTH  = pulseIn(strafePinRC, HIGH);//, PULSEIN_TIMEOUT);

  // If pulses too short, stop robot
  if(DRIVE_PULSE_WIDTH < 500 || TURN_PULSE_WIDTH < 500 || STRAFE_PULSE_WIDTH < 500) {
    allStop();
    return;
  }
  
  // convert RC signals to continuous values from [-1,1]
  driveVal = convertRCtoFloat(DRIVE_PULSE_WIDTH);
  turnVal  = convertRCtoFloat(TURN_PULSE_WIDTH);
  strafeVal = convertRCtoFloat(STRAFE_PULSE_WIDTH);
  
  normalizeVectors();
  
  // set motor speeds and directions //
  motorA.pulse = 0; motorB.pulse = 0; motorC.pulse = 0;
  
  // drive and strafe
  if(getAbsolute(driveVal) > floatDeadband)  { addDrive(driveVal); }
  if(getAbsolute(strafeVal) > floatDeadband) { addStrafe(strafeVal); }
  
  // use turn to control steering
  if(getAbsolute(turnVal) > floatDeadband) { addTurn(turnVal); }
  else                                     { addTurn(0); }
  
  // write high or low to direction pins
  setDirection();
  
  // apply pulse width to PWM outputs
  commandMotors();
  
  Serial.print(driveVal); Serial.print(","); Serial.print(turnVal); Serial.print(",");
  Serial.print(strafeVal); Serial.print("\t");
  Serial.print(motorA.pulse); Serial.print(",");
  Serial.print(motorB.pulse); Serial.print(",");
  Serial.print(motorC.pulse); Serial.print("\n");
}





