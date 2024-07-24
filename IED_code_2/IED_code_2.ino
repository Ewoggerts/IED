#include <PID_v1.h>
#include <TimerOne.h>
#include <L298N.h>
#include <HCSR04.h>


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
const int EncoderLPin = 18; 
const int EncoderRPin = 19; 

// Define ultrasonic sensor pins
const int TrigPin1 = 28;
const int EchoPin1 = 29;
const int TrigPin2 = 30;
const int EchoPin2 = 31;
const int TrigPin3 = 32;
const int EchoPin3 = 33;

// Define IR sensor pins
const int IrSensorLPin = A6;
const int IrSensorRPin = A7;

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
<<<<<<< Updated upstream
PID myPIDRight(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT); 



=======
PID myPIDRight(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);
double maxMotorSpeed = 40; //Max speed of motor in Ticks per Second
MathFunctions 
>>>>>>> Stashed changes
// Encoder variables
volatile long encoderLCount = 0;
volatile long encoderRCount = 0;

// Other variables
const int MaxDistance = 200;  // Maximum distance to check for obstacles
const int SafeDistance = 30;  // Safe distance from obstacles
const int IrThreshold = 500;  // Threshold for IR sensors to detect drop
bool isStopped = false;

/*INTERRUPT FUNCTIONS BELOW*/

// Interrupt service routines for encoders
void encoderLcnt() {
  encoderLCount++;
}

void encoderRcnt() {
  encoderRCount++;
}

/*modify so it updates input*/
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  int rotationL = (encoderLCount / 20);  // divide by number of holes in Disc
  int rotationR = (encoderRCount / 20);  // divide by number of holes in Disc
  encoderLCount = 0;  //  reset counter to zero
  encoderRCount = 0;  //  reset counter to zero

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
  pinMode(TrigPin1, OUTPUT); //Left
  pinMode(EchoPin1, INPUT);
  pinMode(TrigPin2, OUTPUT); //Middle
  pinMode(EchoPin2, INPUT);
  pinMode(TrigPin3, OUTPUT); //Right
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
  SetpointL = 0;  // Desired angle to maintain
  myPIDLeft.SetMode(AUTOMATIC);
  myPIDLeft.SetOutputLimits(-255, 255); //Set output speed limits (in ticks per second)
  SetpointR = 0;  // Desired angle to maintain
  myPIDRight.SetMode(AUTOMATIC);
  myPIDRight.SetOutputLimits(-255, 255);

  /*INTERUPT CODE BELOW*/

  // Initalize Timer and Timer Interupt 
  Timer1.initialize(1000000); // set timer for 1sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer
  
  // Attach Encoder Interrupt
  attachInterrupt(EncoderLPin, encoderLcnt, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(EncoderRPin, encoderRcnt, RISING);  // increase counter when speed sensor pin goes High
  
}

void loop() {
  
}