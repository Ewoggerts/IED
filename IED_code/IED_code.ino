#include <PID_v1.h>

// Define motor pins
const int MotorLPin1 = 1;
const int MotorLPin2 = 2;
const int MotorLSpeedPin = 3;
const int MotorRPin1 = 4;
const int MotorRPin2 = 5;
const int MotorRSpeedPin = 6;

// Define sweeper motor pins
const int SweeperMotorPinL = 7;
const int SweeperMotorPinR = 8;

// Define encoder pins
const int EncoderLPin = 9; // Encoder for Motor 1
const int EncoderRPin = 10; // Encoder for Motor 2

// Define ultrasonic sensor pins
const int TrigPin1 = 11;
const int EchoPin1 = 12;
const int TrigPin2 = 13;
const int EchoPin2 = 18;
const int TrigPin3 = 19;
const int EchoPin3 = 0;

// Define IR sensor pins
const int IrSensorLPin = A0;
const int IrSensorRPin = A1;

// Define stop button pin
const int StopButtonPin = A2;

// Define speaker pin
const int SpeakerPin = A3;

// PID parameter
double Kp = 2, Ki = 5, Kd = 1;
double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;
PID myPIDLeft(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID myPIDRight(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

// Encoder variables
volatile long encoderLCount = 0;
volatile long encoderRCount = 0;

// Other variables
const int MaxDistance = 200;  // Maximum distance to check for obstacles
const int SafeDistance = 30;  // Safe distance from obstacles
const int IrThreshold = 500;  // Threshold for IR sensors to detect drop
bool isStopped = false;

// Variables for feedback loop
double previousDistance = MaxDistance;
double distanceChangeRate = 0;

/*INTERRUPT FUNCTIONS BELOW*/

// Interrupt service routines for encoders
void encoderLcnt() {
  encoder1Count++;
}

void encoderRcnt() {
  encoder2Count++;
}

/*modify so it updates input*/
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  int rotationL = (encoderLCount / 20);  // divide by number of holes in Disc
  int rotationR = (encoderRCount / 20);  // divide by number of holes in Disc
  encoderLCount=0;  //  reset counter to zero
  encoderRCount=0;  //  reset counter to zero

  //debug
  Serial.print("MotorL Speed: "); 
  Serial.print(rotationL,DEC);  
  Serial.print(" Rotation per seconds "); 
  Serial.print(" MotorR Speed: "); 
  Serial.print(rotationR,DEC);  
  Serial.println(" Rotation per seconds"); 
  Serial.println();
  Timer1.attachInterrupt( timerIsr );  //enable the timer
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
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(EncoderLPin), encoderLcnt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderRPin), encoderRcnt, RISING);
  
  // Initialize ultrasonic sensor pins
  pinMode(TrigPin1, OUTPUT);
  pinMode(EchoPin1, INPUT);
  pinMode(TrigPin2, OUTPUT);
  pinMode(EchoPin2, INPUT);
  pinMode(TrigPin3, OUTPUT);
  pinMode(EchoPin3, INPUT);
  
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
  SetpointA = 0;  // Desired angle to maintain
  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetOutputLimits(-255, 255);
  SetpointB = 0;  // Desired angle to maintain
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetOutputLimits(-255, 255);

  /*INTERUPT CODE BELOW*/

  // Initalize Timer and Timer Interupt 
  Timer1.initialize(1000000); // set timer for 1sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer
  
  // Attach Encoder Interrupt
  attachInterrupt(/*change pin*/, encoderLcnt, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(/*change pin*/, encoderRcnt, RISING);  // increase counter when speed sensor pin goes High
  
}

void loop() {
  if (digitalRead(StopButtonPin) == LOW) {
    isStopped = !isStopped;
    delay(500);  // Debounce delay
  }

  if (isStopped) {
    setMotorSpeed(0, 0);
    setSweeperMotors(false);
    noTone(SpeakerPin);
    return;
  } else {
    setSweeperMotors(true);
  }
  
  checkUltrasonicSensors();
  reverseAndTurn(checkIrSensors());
  
  // Update PID input with encoder feedback
  InputA = //Current Speed
  InputB = //Current Speed
  // Update PID
  myPIDL.Compute();
  myPIDR.Compute();
  
  // Set motor speeds based on PID output
  int motorLSpeed = constrain(255 + OutputL, 0, 255);
  int motorRSpeed = constrain(255 + OutputR, 0, 255);
  
  setMotorSpeed(motorLSpeed, motorRSpeed);
  
  // Debugging information
  Serial.print("SetpointL: ");
  Serial.print(SetpointL);
  Serial.print(" InputL: ");
  Serial.print(InputL);
  Serial.print(" OutputL: ");
  Serial.println(OutputL);
  Serial.print("SetpointR: ");
  Serial.print(SetpointR);
  Serial.print(" InputR: ");
  Serial.print(InputR);
  Serial.print(" OutputR: ");
  Serial.println(OutputR);
  Serial.println();
  
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  
  return distance;
}

void checkUltrasonicSensors() {
  long distance1 = readUltrasonic(TrigPin1, EchoPin1); // Left
  long distance2 = readUltrasonic(TrigPin2, EchoPin2); // Middle
  long distance3 = readUltrasonic(TrigPin3, EchoPin3); // Right

  bool left = (distance1 < 30);
  bool middle = (distance2 < 30);
  bool right = (distance3 < 30);

  Serial.print("Distance_L: ")
  Serial.print(distance1);
  Serial.print(" Distance_M: ")
  Serial.print(distance2);
  Serial.print(" Distance_R: ")
  Serial.println(distance3);
  Serial.println();
  
  // Beep frequency based on closest distance
  long closestDistance = min(distance1, min(distance2, distance3));
  if (closestDistance < SafeDistance) {
    tone(SpeakerPin, 1000 - (closestDistance * 15));  // Beep faster as it gets closer
  } else {
    noTone(SpeakerPin);
  }

  /*FIX LOGIC --------------------------------------------------------------------------------*/
  if (left && !middle && !right){
    SetpointA =
    SetpointB = 
  }
  else if (!left && middle && !right){
    SetpointA =
    SetpointB = 
  }
  else if (!left && !middle && right){
    SetpointA =
    SetpointB = 
  }
  else if (left && middle && !right){
    SetpointA =
    SetpointB = 
  }
  else if (left && !middle && right){
    SetpointA =
    SetpointB = 
  }
  else if (!left && middle && right){
    SetpointA =
    SetpointB = 
  }
  else if (left && middle && right){
    SetpointA =
    SetpointB = 
  }
  /*FIX LOGIC --------------------------------------------------------------------------------*/
}

bool checkIrSensors() {
  int irValueL = analogRead(IrSensorLPin);
  int irValueR = analogRead(IrSensorRPin);
  
  Serial.print("Drop_L: ");
  Serial.print(irValueL);
  Serial.print(" Drop_R: ");
  Serial.println(irValueR);
  Serial.println();

  if (irValueL < IrThreshold || irValueR < IrThreshold) { // Drop detected
    return true;
  }
  
  return false;

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

void reverseAndTurn(bool x) {
  if (x == true){
    SetpointA =
    SetpointB = 
  }
}

void setSweeperMotors(bool state) {
  if (state) {
    digitalWrite(SweeperMotorPinL, HIGH);
    digitalWrite(SweeperMotorPinR, LOW);
  } else {
    digitalWrite(SweeperMotorPinL, LOW);
    digitalWrite(SweeperMotorPinR, LOW);
  }
}
