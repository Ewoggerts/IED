#include <PID_v1.h>


// Define motor pins
const int MotorLPin1 = 24;
const int MotorLPin2 = 25;
const int MotorLSpeedPin = 45;
const int MotorRPin1 = 22;
const int MotorRPin2 = 23;
const int MotorRSpeedPin = 44;

const int MotorLPin1_dup = 36;
const int MotorLPin2_dup = 37;
const int MotorLSpeedPin_dup = 46;
const int MotorRPin1_dup = 38;
const int MotorRPin2_dup = 39;
const int MotorRSpeedPin_dup = 4;

// Define sweeper motor pins
const int SweeperMotorPinL = 26;
const int SweeperMotorPinR = 27;

const int SweeperMotorPinL_dup = 40;
const int SweeperMotorPinR_dup = 41;

// Define encoder pins
const int EncoderLPin = 20;
const int EncoderRPin = 21;

// Define ultrasonic sensor pins
const int UltraSonicTrigLeft = 28;
const int UltraSonicTrigMid = 29;
const int UltraSonicTrigRight = 30;
const int UltraSonicEchoLeft = 31;
const int UltraSonicEchoMid = 32;
const int UltraSonicEchoRight = 33;

// Define IR sensor pins
const int IrSensorLPin = 2;
const int IrSensorRPin = 3;

// Stop button pin
const int StopButtonPin = 34; // Updated for Mega

// Speaker pin
const int SpeakerPin = 35; // Updated for Mega

// PID parameters
double Kp = 0.3, Ki = 0, Kd = 0;
double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;

//inputs are ticks recorded
//output is in ticks in relations to a proportion (ticks per second);
//setpoint is the total ticks that we want
PID myPIDLeft(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID myPIDRight(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);
double maxMotorSpeed = 100; //Max speed of motor in Ticks per Second

// Encoder variables
volatile long encoderLCount = 0;
volatile long encoderRCount = 0;

// Other variables
const int SafeDistance = 25;  // Safe distance from obstacles
bool isStopped = false;
int wheelDiameter = 5; //cm
int ticksPerRev = 20;
int maxTicksPerSec = 100;
int stopMargin = 5;
float driveBase = 18.5;
int leftPWM = 0;
int rightPWM = 0;

//flag
int stoppedL = 0;
int stoppedR = 0;
int reverseL = 0;
int reverseR = 0;
int dropFlag = 0;

/*INTERRUPT FUNCTIONS BELOW*/

// Interrupt service routines for encoders
void encoderLcnt() {
  if (reverseL){
    encoderLCount--;
    //Serial.println();
    //Serial.println("L--");
  }
  else{
    encoderLCount++;
    //Serial.println();
    //Serial.println("L++");
  }
  //debug
  //Serial.print("encoderLCount: ");
  //Serial.println(encoderLCount);
}

void encoderRcnt() {
  if (reverseR){
    encoderRCount--;
  }
  else{
    encoderRCount++;
  }
  //debug
  //Serial.print("encoderRCount: ");
  //Serial.println(encoderRCount);
}

void setup() {
  // Initialize motor pins
  pinMode(MotorLPin1, OUTPUT);
  pinMode(MotorLPin2, OUTPUT);
  pinMode(MotorLSpeedPin, OUTPUT);
  pinMode(MotorRPin1, OUTPUT);
  pinMode(MotorRPin2, OUTPUT);
  pinMode(MotorRSpeedPin, OUTPUT);

  pinMode(MotorLPin1_dup, OUTPUT);
  pinMode(MotorLPin2_dup, OUTPUT);
  pinMode(MotorLSpeedPin_dup, OUTPUT);
  pinMode(MotorRPin1_dup, OUTPUT);
  pinMode(MotorRPin2_dup, OUTPUT);
  pinMode(MotorRSpeedPin_dup, OUTPUT);

  // Initialize sweeper motor pins
  pinMode(SweeperMotorPinL, OUTPUT);
  pinMode(SweeperMotorPinR, OUTPUT);
  
  pinMode(SweeperMotorPinL_dup, OUTPUT);
  pinMode(SweeperMotorPinR_dup, OUTPUT);

  // Initialize encoder pins
  pinMode(EncoderLPin, INPUT);
  pinMode(EncoderRPin, INPUT);

  // Initialize ultrasonic sensor pins
  pinMode(UltraSonicTrigLeft, OUTPUT);
  pinMode(UltraSonicTrigMid, OUTPUT);
  pinMode(UltraSonicTrigRight, OUTPUT);
  pinMode(UltraSonicEchoLeft, INPUT);
  pinMode(UltraSonicEchoMid, INPUT);
  pinMode(UltraSonicEchoRight, INPUT);

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
  attachInterrupt(digitalPinToInterrupt(IrSensorLPin), dropAvoidance, RISING);
  attachInterrupt(digitalPinToInterrupt(IrSensorRPin), dropAvoidance, RISING);

  //sweeper motors
  digitalWrite(SweeperMotorPinL, HIGH);
  digitalWrite(SweeperMotorPinR, LOW);
  
  digitalWrite(SweeperMotorPinL_dup, HIGH);
  digitalWrite(SweeperMotorPinR_dup, LOW);


  // Give some time to set the car down
  delay(2000);

  // Initialize PID to start moving the car forward
  SetpointL = distanceToWheelRev(300, wheelDiameter, ticksPerRev);  // Initialize Desired distance to reach
  myPIDLeft.SetMode(AUTOMATIC);
  myPIDLeft.SetOutputLimits(-maxTicksPerSec, maxTicksPerSec); // Set output speed limits (in ticks per second)
  SetpointR = distanceToWheelRev(300, wheelDiameter, ticksPerRev);  // Initialize Desired distance to reach
  myPIDRight.SetMode(AUTOMATIC);
  myPIDRight.SetOutputLimits(-maxTicksPerSec, maxTicksPerSec); // Set output speed limits (in ticks per second)

  // Debugging info
  Serial.println("SETUP COMPLETE");

  long* data = readAllDistances();
  obstacleAvoidance(data); // Constantly checks for obstacles
}

void loop() {
  // MAINLOOP LOGIC DEBUG
  Serial.println("MAINLOOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  //Keep moving forward if this happens
  if (SetpointL == 0 && SetpointR == 0 && stoppedL && stoppedR){
    delay(500);
    changeDirection(false);
    drive(100);
    stoppedL = 0;
    stoppedR = 0;
  }

  //Handles drop detection
 /* if (dropFlag){
    Serial.println();
    Serial.println("DROP DETECTED -----------------------------------------------------------");
    drive(-25); // reverse
    forceWait(stopMargin);
    Serial.println("EMERGENCY FINISHED");
    changeDirection(true); 
    delay(1000);
    dropFlag = 0;
    Serial.println("DROP AVOIDED -----------------------------------------------------------");
  }*/
  
  long* data = readAllDistances();
  obstacleAvoidance(data); // Constantly checks for obstacles

  /* PID ------------------------------------------------------------------ */
  InputL = encoderLCount;
  InputR = encoderRCount;
  // Left Motor
  if (!stoppedL){
    Serial.println("PID-LEFT");
    myPIDLeft.Compute();
    leftPWM = normalizeToPWM(maxTicksPerSec, OutputL);  // Fixed to use OutputL
    setMotorSpeedL(leftPWM);
  }
  // Right Motor
  if (!stoppedR){
    Serial.println("PID-RIGHT");
    myPIDRight.Compute();
    rightPWM = normalizeToPWM(maxTicksPerSec, OutputR); // Fixed to use OutputR
    setMotorSpeedR(rightPWM);
  }
  /* PID ------------------------------------------------------------------ */

  // Debugging info for PID
  Serial.print("ReverseLFlag: ");
  Serial.print(reverseL);
  Serial.print(" ReverseRFlag: ");
  Serial.println(reverseR);

  Serial.print("Loop - InputL: ");
  Serial.print(InputL);
  Serial.print(" InputR: ");
  Serial.print(InputR);
  Serial.print(" OutputL: ");
  Serial.print(OutputL);
  Serial.print(" OutputR: ");
  Serial.print(OutputR);
  Serial.print(" SetpointL: ");
  Serial.print(SetpointL);
  Serial.print(" SetpointR: ");
  Serial.println(SetpointR);

  stopPoint(OutputL, OutputR); //Logic used to stop once pid has reach desired state

  // Debugging info for stopping PID
  Serial.print("StopLFlag: ");
  Serial.print(stoppedL);
  Serial.print(" StopRFlag: ");
  Serial.println(stoppedR);
  Serial.println();
  
}