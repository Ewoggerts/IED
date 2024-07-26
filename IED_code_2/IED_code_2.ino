#include <PID_v1.h>
#include <TimerOne.h>
#include <L298N.h>
#include <HCSR04.h>
#include <Functions.ino>

// Define motor pins
const int MotorLPin1 = 22;
const int MotorLPin2 = 23;
const int MotorLSpeedPin = 2;
const int MotorRPin1 = 24;
const int MotorRPin2 = 25;
const int MotorRSpeedPin = 3;

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

/*INTERRUPT FUNCTIONS BELOW*/

// Interrupt service routines for encoders
void encoderLcnt() {
  encoderLCount++;
  //debug
  Serial.print("encoderLCount: ");
  Serial.println(encoderLcnt);
}

void encoderRcnt() {
  encoderRCount++;
  //debug
  Serial.print("encoderRCount: ");
  Serial.println(encoderRcnt);
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
  
  // Initialize stop button pin
  pinMode(StopButtonPin, INPUT_PULLUP);

  // Initialize speaker pin
  pinMode(SpeakerPin, OUTPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize PID
  SetpointL = 0;  // Desired angle to maintain
  myPIDLeft.SetMode(AUTOMATIC);
  myPIDLeft.SetOutputLimits(-maxTicksPerSec, maxTicksPerSec); //Set output speed limits (in ticks per second)
  SetpointR = 0;  // Desired angle to maintain
  myPIDRight.SetMode(AUTOMATIC);
  myPIDRight.SetOutputLimits(-maxTicksPerSec, maxTicksPerSec); //Set output speed limits (in ticks per second)

  /*ATTACH INTERUPT BELOW*/
  attachInterrupt(digitalPinToInterrupt(EncoderLPin), encoderLcnt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderRPin), encoderRcnt, RISING);
  attachInterrupt(digitalPinToInterrupt(IrSensorLPin), dropAvoidance, FALLING);
  attachInterrupt(digitalPinToInterrupt(IrSensorRPin), dropAvoidance, FALLING);  
}

void loop() {
  obstacleAvoidance(HCSR04.measureDistanceCm()); //Constantly checks for need direction change
}

void obstacleAvoidance( double* distances){
  //Returns a boolean that determines if safeDistance has been breached
  if (checkDist(distances, SafeDistance)){
    changeDirection(false); //Random direction change without a drop
  }
}

void dropAvoidance() {
  shortReverse(); //10 cm reverse
  changeDirection(true); //180 direction change if there is a drop
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

  //set encoders back to 0 for pid 
  encoderLcnt = 0;
  encoderRcnt = 0;

  //Determine which wheel goes back or forward
  if (deg < 0){
    SetpointL = -ticks;
    SetpointR = ticks;
  }
  else{
    SetpointL = ticks;
    SetpointR = -ticks;
  }
}

void shortReverse(){
  //set encoders back to 0 for pid 
  encoderLcnt = 0;
  encoderRcnt = 0;

  //10cm reverse 
  int shortReverse = distanceToWheelRev(10 /*cm*/, wheelDiameter, ticksPerRev);

  //Set for reversal
  SetpointL = -shortReverse;
  SetpointR = -shortReverse;
  
  //Normalize pid output to pwm signal
  int revLeft = normalizeToPWM(OutputL);
  int revRight = normalizeToPWM(OutputR);

  forceWait();
  
}

void forceWait(){
  //Force wait till reverse is complete
  while (encoderLcnt != SetpointL ||  encoderRcnt != SetpointR){
    inputL = encoderLcnt;
    inputR = encoderRcnt;
    myPIDLeft.Compute();
    myPIDRight.compute();
  }
}

void driveForward(int desiredDist){

}