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
double Kp = 2, Ki = 5, Kd = 1;
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
const int SafeDistance = 55;  // Safe distance from obstacles
bool isStopped = false;
int wheelDiameter = 5; //cm
int ticksPerRev = 20;
int maxTicksPerSec = 100;
int stopMargin = 15;
float driveBase = 18.5;
int leftPWM = 0;
int rightPWM = 0;

//flag
int stoppedL = 0;
int stoppedR = 0;

/*INTERRUPT FUNCTIONS BELOW*/

// Interrupt service routines for encoders
void encoderLcnt() {
  encoderLCount++;
  //debug
  //Serial.print("encoderLCount: ");
  //Serial.println(encoderLCount);
}

void encoderRcnt() {
  encoderRCount++;
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
  attachInterrupt(digitalPinToInterrupt(IrSensorLPin), dropAvoidance, FALLING);
  attachInterrupt(digitalPinToInterrupt(IrSensorRPin), dropAvoidance, FALLING);

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
  Serial.println("Setup completed.");
}

void loop() {

  
  Serial.println("MAINLOOP!!!!");
  long* data = readAllDistances();
  //obstacleAvoidance(data); // Constantly checks for the need for direction change

  /* PID ------------------------------------------------------------------ */
  InputL = encoderLCount;
  InputR = encoderRCount;
 

  if (!stoppedL){
    Serial.println("PID-LEFT");
    myPIDLeft.Compute();
    leftPWM = normalizeToPWM(maxTicksPerSec, OutputL);  // Fixed to use OutputL
    setMotorSpeedL(leftPWM);
  }
  if (!stoppedR){
    Serial.println("PID-RIGHT");
    myPIDRight.Compute();
    rightPWM = normalizeToPWM(maxTicksPerSec, OutputR); // Fixed to use OutputR
    setMotorSpeedR(rightPWM);
  }
 
  /* PID ------------------------------------------------------------------ */

  // Debugging info  
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
  

  stopPoint(OutputL, OutputR);

  Serial.print("StopLFlag: ");
  Serial.print(stoppedL);
  Serial.print(" StopRFlag: ");
  Serial.println(stoppedR);
  Serial.println();
  
}

void obstacleAvoidance(long* distances) {
  // Returns a boolean that determines if safeDistance has been breached 
  if (checkDist(distances, SafeDistance)) {
    Serial.println();
    Serial.println("SAFE DIST BREACH ---------------------------------------------------------");
    changeDirection(false); // Random direction change without a drop
    drive(200);
    Serial.println("SAFE DIST RESET ----------------------------------------------------------");
  }
}

void dropAvoidance() {
  Serial.println("Drop detected by IR sensors.");
  drive(-25); // 10 cm reverse
  Serial.println("EMERGENCY REVERSING");
  forceWait(stopMargin);
  Serial.println("EMERGENCY FINISHED");
  // Uncomment if 180 direction change is needed
  changeDirection(true); // 180 direction change if there is a drop
}

void forceWait(int margin) {
  // Debugging info
  Serial.println("FORCE-WAIT activated.");

  // Force wait till adjustment outputs are really small
  while (!stoppedL || !stoppedR) {
    /* PID ------------------------------------------------------------------ */
    
    Serial.println("forced wait loop ... ...");
    InputL = encoderLCount;
    InputR = encoderRCount;

    if (!stoppedL){
      Serial.println("PID-LEFT");
      myPIDLeft.Compute();
      leftPWM = normalizeToPWM(maxTicksPerSec, OutputL);  // Fixed to use OutputL
      setMotorSpeedL(leftPWM);
    }
    if (!stoppedR){
      Serial.println("PID-RIGHT");
      myPIDRight.Compute();
      rightPWM = normalizeToPWM(maxTicksPerSec, OutputR); // Fixed to use OutputR
      setMotorSpeedR(rightPWM);
    }
    /* PID ------------------------------------------------------------------ */

      // Debugging info  
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
    
    
    stopPoint(OutputL, OutputR);
    
    Serial.print("StopLFlag: ");
    Serial.print(stoppedL);
    Serial.print(" StopRFlag: ");
    Serial.println(stoppedR);
    Serial.println();
  }

  // Debugging info
  Serial.println("FORCE-WAIT completed.");
}

void drive(int desiredDist) {
  Serial.print("Driving forwards or backwards by ");
  Serial.print(desiredDist);
  Serial.println(" cm.");
  stoppedL = 0;
  stoppedR = 0;
  // Set encoders back to 0 for PID
  encoderLCount = 0;
  encoderRCount = 0;

  // Forward set distance
  int driveDist = distanceToWheelRev(desiredDist, wheelDiameter, ticksPerRev);

  // Set for driving forward
  if (desiredDist > 0) {
    SetpointL = driveDist;
    SetpointR = driveDist;
  }
  else { // Set for driving reverse
    SetpointL = -driveDist;
    SetpointR = -driveDist;
  }

  // Debugging info
  Serial.print("Drive SetpointL: ");
  Serial.print(SetpointL);
  Serial.print(" SetpointR: ");
  Serial.println(SetpointR);
}

void changeDirection(bool forced) {
  Serial.println("Change Direction Loop Activated");

  // Forced 180 direction change (in cases where there is a drop)
  int deg = 180;
  int ticks = turnAngleToWheelRev(deg, driveBase, wheelDiameter, ticksPerRev);

  // Otherwise random direction change
  if (!forced) {
    deg = generateRandomValue();
    ticks = turnAngleToWheelRev(deg, driveBase, wheelDiameter, ticksPerRev);
  }

  // Debugging info
  Serial.print("Turning angle degrees: ");
  Serial.println(deg);

  // Make alert sound
  tone(SpeakerPin, 1000);

  // Determine which wheel goes back or forward
  if (deg < 0) {
    SetpointL = -ticks;
    SetpointR = ticks;
  }
  else {
    SetpointL = ticks;
    SetpointR = -ticks;
  }

  stoppedL = 0;
  stoppedR = 0;

  encoderLCount = 0;
  encoderRCount = 0;

  // Debugging info
  Serial.print("Change Direction SetpointL: ");
  Serial.print(SetpointL);
  Serial.print(" SetpointR: ");
  Serial.println(SetpointR);

  forceWait(5);

  // End alert sound after direction change finished
  noTone(SpeakerPin);
  // Debugging info
  Serial.println("Direction change completed.");
}

void setMotorSpeedL(int motorLSpeed) {
  // Debugging info
  Serial.print("Setting motor speeds - Left: ");
  Serial.println(motorLSpeed);

  if (motorLSpeed > 0) {
    digitalWrite(MotorLPin1, HIGH);
    digitalWrite(MotorLPin2, LOW);
    digitalWrite(MotorLPin1_dup, HIGH);
    digitalWrite(MotorLPin2_dup, LOW);
  } else {
    digitalWrite(MotorLPin1, LOW);
    digitalWrite(MotorLPin2, HIGH);
    digitalWrite(MotorLPin1_dup, LOW);
    digitalWrite(MotorLPin2_dup, HIGH);
    motorLSpeed = -motorLSpeed;
  }
  analogWrite(MotorLSpeedPin, motorLSpeed);
  analogWrite(MotorLSpeedPin_dup, motorLSpeed);
  
}

void setMotorSpeedR(int motorRSpeed) {
  // Debugging info
  Serial.print("Setting motor speeds - Right: ");
  Serial.println(motorRSpeed);
  
  if (motorRSpeed > 0) {
    digitalWrite(MotorRPin1, HIGH);
    digitalWrite(MotorRPin2, LOW);
    digitalWrite(MotorRPin1_dup, HIGH);
    digitalWrite(MotorRPin2_dup, LOW);
  } else {
    digitalWrite(MotorRPin1, LOW);
    digitalWrite(MotorRPin2, HIGH);
    digitalWrite(MotorRPin1_dup, LOW);
    digitalWrite(MotorRPin2_dup, HIGH);
    motorRSpeed = -motorRSpeed;
  }
  analogWrite(MotorRSpeedPin, motorRSpeed);
  analogWrite(MotorRSpeedPin_dup, motorRSpeed);
}

void stopPoint(int setpL, int setpR) {
  // Debugging info
  Serial.print("Output - Left: ");
  Serial.print(setpL);
  Serial.print(", Right: ");
  Serial.println(setpR);

  if (setpL < stopMargin) {
    Serial.println("STOP-LEFT");
    analogWrite(MotorLSpeedPin, 0);
    encoderLCount = 0;
    SetpointL = 0;
    stoppedL = 1;
  }

  if (setpR < stopMargin) {
    Serial.println("STOP-RIGHT");
    analogWrite(MotorRSpeedPin, 0);
    encoderRCount = 0;
    SetpointR = 0;
    stoppedR = 1;
  }
}

long* readAllDistances() {
  static long distances[3];

  // Trigger and read left sensor
  distances[0] = getDistance(UltraSonicTrigLeft, UltraSonicEchoLeft);

  // Trigger and read middle sensor
  distances[1] = getDistance(UltraSonicTrigMid, UltraSonicEchoMid);

  // Trigger and read right sensor
  distances[2] = getDistance(UltraSonicTrigRight, UltraSonicEchoRight);

  // Print distances
  Serial.print("Left: ");
  Serial.print(distances[0]);
  Serial.print(" cm, Mid: ");
  Serial.print(distances[1]);
  Serial.print(" cm, Right: ");
  Serial.println(distances[2]);

  return distances;
}