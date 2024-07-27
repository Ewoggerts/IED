#include <PID_v1.h>
#include <TimerOne.h>
#include <L298N.h>
#include <HCSR04.h>
#include <Functions.ino>

// Define motor pins
const int MotorLPin1 = 22;
const int MotorLPin2 = 23;
const int MotorLSpeedPin = 44;
const int MotorRPin1 = 24;
const int MotorRPin2 = 25;
const int MotorRSpeedPin = 45;



// Define sweeper motor pins
const int SweeperMotorPinL = 26;
const int SweeperMotorPinR = 27;

// Define encoder pins
const int EncoderLPin = 20; 
const int EncoderRPin = 21; 

// Define ultrasonic sensor pins
byte triggerPin = 28;
byte echoCount = 3;
byte* echoPins = new byte[3] { 29, 30, 31 };

// Define IR sensor pins
const int IrSensorLPin = 2;
const int IrSensorRPin = 3;

// Stop button pin
const int StopButtonPin = 34; // Updated for Mega

// Speaker pin
const int SpeakerPin = 35; // Updated for Mega

// PID parameter
double Kp = 2, Ki = 5, Kd = 1;
double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;

//inputs are ticks recorded
//output is in ticks in relations to a proportion (ticks per second);
//setpoint is the total ticks that we want
PID myPIDLeft(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID myPIDRight(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);
double maxMotorSpeed = 40; //Max speed of motor in Ticks per Second
 
// Encoder variables
volatile long encoderLCount = 0;
volatile long encoderRCount = 0;

// Other variables
const int SafeDistance = 30;  // Safe distance from obstacles
bool isStopped = false;
int wheelDiameter = 6; //cm
int ticksPerRev = 20;
int maxTicksPerSec = 7;
int stopMargin = 3;
float driveBase = 5.5;
/*INTERRUPT FUNCTIONS BELOW*/

// Interrupt service routines for encoders
void encoderLcnt() {
  encoderLCount++;
  //debug
  Serial.print("encoderLCount: ");
  Serial.println(encoderLCount);
}

void encoderRcnt() {
  encoderRCount++;
  //debug
  Serial.print("encoderRCount: ");
  Serial.println(encoderRCount);
}

void setup() {
  // Initialize motor pins
  pinMode(MotorLPin1, OUTPUT);
  pinMode(MotorLPin2, OUTPUT);
  pinMode(MotorLSpeedPin, OUTPUT);
  pinMode(MotorRPin1, OUTPUT);
  pinMode(MotorRPin2, OUTPUT);
  pinMode(MotorRSpeedPin, OUTPUT);

  // Initialize sweeper motor pins
  pinMode(SweeperMotorPinL, OUTPUT);
  pinMode(SweeperMotorPinR, OUTPUT);
  
  // Initialize encoder pins
  pinMode(EncoderLPin, INPUT);
  pinMode(EncoderRPin, INPUT);
  
  // Initialize ultrasonic sensor pins
  HCSR04.begin(triggerPin, echoPins, echoCount);
  
  // Initialize IR sensor pins
  pinMode(IrSensorLPin, INPUT);
  pinMode(IrSensorRPin, INPUT);

  // Initialize speaker pin
  pinMode(SpeakerPin, OUTPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(EncoderLPin), encoderLcnt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderRPin), encoderRcnt, RISING);
  attachInterrupt(digitalPinToInterrupt(IrSensorLPin), dropAvoidance, FALLING);
  attachInterrupt(digitalPinToInterrupt(IrSensorRPin), dropAvoidance, FALLING);  
  
  //Give some time to set the car down
  delay(1000);

  // Initialize PID to starting moving the car forward
  SetpointL = distanceToWheelRev( 40, wheelDiameter, ticksPerRev);  // Intialized Desired distance to reach
  myPIDLeft.SetMode(AUTOMATIC);
  myPIDLeft.SetOutputLimits(-maxTicksPerSec, maxTicksPerSec); //Set output speed limits (in ticks per second)
  SetpointR = distanceToWheelRev( 40, wheelDiameter, ticksPerRev);  // Intialized Desired distance to reach
  myPIDRight.SetMode(AUTOMATIC);
  myPIDRight.SetOutputLimits(-maxTicksPerSec, maxTicksPerSec); //Set output speed limits (in ticks per second)

  // Start the sweeper motors
  digitalWrite(SweeperMotorPinL, HIGH);
  digitalWrite(SweeperMotorPinR, HIGH);
}

void loop() {
  obstacleAvoidance(HCSR04.measureDistanceCm()); //Constantly checks for need direction change
  /*PID ------------------------------------------------------------------*/
  InputL = encoderLCount;
  InputR = encoderRCount;
  myPIDLeft.Compute();
  myPIDRight.Compute();
  int leftPWM = normalizeToPWM(maxTicksPerSec, maxTicksPerSec);
  int rightPWM = normalizeToPWM(maxTicksPerSec, maxTicksPerSec);
  setMotorSpeed(leftPWM, rightPWM);
  /*PID ------------------------------------------------------------------*/
  //Determines if the car has stopped and reach it desired distance
  if (OutputL <= 3 && OutputR <= 3){
    changeDirection(false);
    drive(45); //sets the car to keep driving forward 45cm until another interrupt or distance reached
  }

}

void obstacleAvoidance( double* distances){
  //Returns a boolean that determines if safeDistance has been breached
  if (checkDist(distances, SafeDistance)){
    changeDirection(false); //Random direction change without a drop
  }
  drive(45);
}

void dropAvoidance() {
  drive(-10); //10 cm reverse
  changeDirection(true); //180 direction change if there is a drop
}

void forceWait(int margin){
  //Force wait till adjustment outputs are really small
  while (OutputL <= margin && OutputR <= margin){
    /*PID ------------------------------------------------------------------*/
    InputL = encoderLCount;
    InputR = encoderRCount;
    myPIDLeft.Compute();
    myPIDRight.Compute();
    int leftPWM = normalizeToPWM(maxTicksPerSec, maxTicksPerSec);
    int rightPWM = normalizeToPWM(maxTicksPerSec, maxTicksPerSec);
    setMotorSpeed(leftPWM, rightPWM);
    /*PID ------------------------------------------------------------------*/
  }
}

void drive(int desiredDist){
  //set encoders back to 0 for pid 
  encoderLCount = 0;
  encoderRCount = 0;

  //forward set dist 
  int driveDist = distanceToWheelRev(desiredDist, wheelDiameter, ticksPerRev);

  //Set for driving forward
  if (desiredDist > 0){
    SetpointL = driveDist;
    SetpointR = driveDist;
  }
  else{ //Set for driving reverse
    SetpointL = -driveDist;
    SetpointR = -driveDist;
  }
}

void changeDirection(bool forced){
  //Forced 180 direction change (in cases where there is a drop)
  int deg = 180;
  int ticks = turnAngleToWheelRev(deg, driveBase, wheelDiameter, ticksPerRev);

  //Otherwise random direction change
  if (!forced){
    deg = generateRandomValue(-180, 180);
    ticks = turnAngleToWheelRev(deg, driveBase, wheelDiameter, ticksPerRev);
  }

  //Beep to alert close to an object 
  tone(SpeakerPin, 200);

  //Set encoders back to 0 for pid 
  encoderLCount = 0;
  encoderRCount = 0;

  //Determine which wheel goes back or forward
  if (deg < 0){
    SetpointL = -ticks;
    SetpointR = ticks;
  }
  else{
    SetpointL = ticks;
    SetpointR = -ticks;
  }

  forceWait(stopMargin);

  //End alert sound after direction change finished
  noTone(SpeakerPin);
}

void setMotorSpeed(int motorLSpeed, int motorRSpeed) {
  if (motorLSpeed > 0) {
    digitalWrite(MotorLPin1, HIGH);
    digitalWrite(MotorLPin2, LOW);
  } else {
    digitalWrite(MotorLPin1, LOW);
    digitalWrite(MotorLPin2, HIGH);
    motorLSpeed = -motorLSpeed;
  }
  
  if (motorRSpeed > 0) {
    digitalWrite(MotorRPin1, HIGH);
    digitalWrite(MotorRPin2, LOW);
  } else {
    digitalWrite(MotorRPin1, LOW);
    digitalWrite(MotorRPin2, HIGH);
    motorRSpeed = -motorRSpeed;
  }
  
  analogWrite(MotorLSpeedPin, motorRSpeed);
  analogWrite(MotorRSpeedPin, motorRSpeed);
}
