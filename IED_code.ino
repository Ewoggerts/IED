#include <PID_v1.h>

// Define motor pins
const int Motor1Pin1 = 1;
const int Motor1Pin2 = 2;
const int Motor1SpeedPin = 3;
const int Motor2Pin1 = 4;
const int Motor2Pin2 = 5;
const int Motor2SpeedPin = 6;

// Define sweeper motor pins
const int SweeperMotorPin1 = 7;
const int SweeperMotorPin2 = 8;

// Define encoder pins
const int Encoder1Pin = 9; // Encoder for Motor 1
const int Encoder2Pin = 10; // Encoder for Motor 2

// Define ultrasonic sensor pins
const int TrigPin1 = 11;
const int EchoPin1 = 12;
const int TrigPin2 = 13;
const int EchoPin2 = 18;
const int TrigPin3 = 19;
const int EchoPin3 = 0;

// Define IR sensor pins
const int IrSensor1Pin = A0;
const int IrSensor2Pin = A1;

// Define stop button pin
const int StopButtonPin = A2;

// Define speaker pin
const int SpeakerPin = A3;

// PID parameter
double Kp = 2, Ki = 5, Kd = 1;
double SetpointA, InputA, OutputA;
double SetpointB, InputB, OutputB;
PID myPID1(&InputA, &OutputA, &SetpointA, Kp, Ki, Kd, DIRECT);
PID myPID2(&InputB, &OutputB, &SetpointB, Kp, Ki, Kd, DIRECT);

// Encoder variables
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

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
void encoder1cnt() {
  encoder1Count++;
}

void encoder2cnt() {
  encoder2Count++;
}

/*modify so it updates input*/
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  int rotation1 = (encoder1Count / 20);  // divide by number of holes in Disc
  int rotation2 = (encoder2Count / 20);  // divide by number of holes in Disc
  encoder1Count=0;  //  reset counter to zero
  encoder2Count=0;  //  reset counter to zero

  //debug
  Serial.print("Motor1 Speed: "); 
  Serial.print(rotation,DEC);  
  Serial.print(" Rotation per seconds "); 
  Serial.print(" Motor2 Speed: "); 
  Serial.print(rotation,DEC);  
  Serial.println(" Rotation per seconds"); 
  Serial.println();
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

void setup() {
  // Initialize motor pins
  pinMode(Motor1Pin1, OUTPUT);
  pinMode(Motor1Pin2, OUTPUT);
  pinMode(Motor1SpeedPin, OUTPUT);
  pinMode(Motor2Pin1, OUTPUT);
  pinMode(Motor2Pin2, OUTPUT);
  pinMode(Motor2SpeedPin, OUTPUT);

  // Initialize sweeper motor pins
  pinMode(SweeperMotorPin1, OUTPUT);
  pinMode(SweeperMotorPin2, OUTPUT);
  
  // Initialize encoder pins
  pinMode(Encoder1Pin, INPUT);
  pinMode(Encoder2Pin, INPUT);
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(Encoder1Pin), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder2Pin), encoder2ISR, RISING);
  
  // Initialize ultrasonic sensor pins
  pinMode(TrigPin1, OUTPUT);
  pinMode(EchoPin1, INPUT);
  pinMode(TrigPin2, OUTPUT);
  pinMode(EchoPin2, INPUT);
  pinMode(TrigPin3, OUTPUT);
  pinMode(EchoPin3, INPUT);
  
  // Initialize IR sensor pins
  pinMode(IrSensor1Pin, INPUT);
  pinMode(IrSensor2Pin, INPUT);
  
  // Initialize stop button pin
  pinMode(StopButtonPin, INPUT_PULLUP);

  // Initialize speaker pin
  pinMode(SpeakerPin, OUTPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize PID
  SetpointA = 0;  // Desired angle to maintain
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-255, 255);
  SetpointB = 0;  // Desired angle to maintain
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-255, 255);

  /*INTERUPT CODE BELOW*/

  // Initalize Timer and Timer Interupt 
  Timer1.initialize(1000000); // set timer for 1sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer
  
  // Attach Encoder Interrupt
  attachInterrupt(/*change pin*/, encoder1cnt, RISING);  // increase counter when speed sensor pin goes High
  attachInterrupt(/*change pin*/, encoder2cnt, RISING);  // increase counter when speed sensor pin goes High
  
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
  myPID1.Compute();
  myPID2.Compute();
  
  // Set motor speeds based on PID output
  int motor1Speed = constrain(255 + OutputA, 0, 255);
  int motor2Speed = constrain(255 + OutputB, 0, 255);
  
  setMotorSpeed(motor1Speed, motor2Speed);
  
  // Debugging information
  Serial.print("SetpointA: ");
  Serial.print(SetpointA);
  Serial.print(" InputA: ");
  Serial.print(InputA);
  Serial.print(" OutputA: ");
  Serial.println(OutputA);
  Serial.print("SetpointB: ");
  Serial.print(SetpointB);
  Serial.print(" InputB: ");
  Serial.print(InputB);
  Serial.print(" OutputB: ");
  Serial.println(OutputB);
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
    tone(SpeakerPin, 1000 - (closestDistance * 10));  // Beep faster as it gets closer
  } else {
    noTone(SpeakerPin);
  }

  /*FIX LOGIC -------------------------------------------------------------------------------*/
  
  /*FIX LOGIC --------------------------------------------------------------------------------*/
}

bool checkIrSensors() {
  int irValue1 = analogRead(IrSensor1Pin);
  int irValue2 = analogRead(IrSensor2Pin);
  
  Serial.print("Drop_L: ");
  Serial.print(irValue1);
  Serial.print(" Drop_R: ");
  Serial.println(irValue2);
  Serial.println();

  if (irValue1 < IrThreshold || irValue2 < IrThreshold) { // Drop detected
    return true;
  }
  
  return false;


}

void setMotorSpeed(int motor1Speed, int motor2Speed) {
  if (motor1Speed > 0) {
    digitalWrite(Motor1Pin1, HIGH);
    digitalWrite(Motor1Pin2, LOW);
  } else {
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor1Pin2, HIGH);
    motor1Speed = -motor1Speed;
  }
  
  if (motor2Speed > 0) {
    digitalWrite(Motor2Pin1, HIGH);
    digitalWrite(Motor2Pin2, LOW);
  } else {
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor2Pin2, HIGH);
    motor2Speed = -motor2Speed;
  }
  
  analogWrite(Motor1SpeedPin, motor1Speed);
  analogWrite(Motor2SpeedPin, motor2Speed);
}

void reverseAndTurn(bool x) {
  if (x == true){
    //reverse a little bit
    //spin car around
  }
}

void setSweeperMotors(bool state) {
  if (state) {
    digitalWrite(SweeperMotorPin1, HIGH);
    digitalWrite(SweeperMotorPin2, LOW);
  } else {
    digitalWrite(SweeperMotorPin1, LOW);
    digitalWrite(SweeperMotorPin2, LOW);
  }
}
