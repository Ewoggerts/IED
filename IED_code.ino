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

// PID parameters
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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

// Interrupt service routines for encoders
void encoder1ISR() {
  encoder1Count++;
}

void encoder2ISR() {
  encoder2Count++;
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
  Setpoint = 0;  // Desired angle to maintain
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
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
  
  if (checkIrSensors()) {
    // Reverse and turn if a drop is detected
    reverseAndTurn();
    return;
  }
  
  // Update PID input with encoder feedback
  Input = (encoder1Count - encoder2Count) / 2.0;
  
  // Update PID
  myPID.Compute();
  
  // Set motor speeds based on PID output
  int motor1Speed = constrain(255 + Output, 0, 255);
  int motor2Speed = constrain(255 - Output, 0, 255);
  
  setMotorSpeed(motor1Speed, motor2Speed);
  
  // Debugging information
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" Input: ");
  Serial.print(Input);
  Serial.print(" Output: ");
  Serial.println(Output);
  
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
  
  // Beep frequency based on closest distance
  long closestDistance = min(distance1, min(distance2, distance3));
  if (closestDistance < SafeDistance) {
    tone(SpeakerPin, 1000 - (closestDistance * 10));  // Beep faster as it gets closer
  } else {
    noTone(SpeakerPin);
  }

  if (closestDistance < SafeDistance) {
    // Calculate rate of distance change
    distanceChangeRate = previousDistance - closestDistance;
    previousDistance = closestDistance;

    // Adjust setpoint for PID based on obstacle position, distance, and rate of change
    if (distance2 < SafeDistance) {
      // If the middle sensor detects an obstacle, move left
      Setpoint = map(closestDistance, 0, SafeDistance, -90, -30) - distanceChangeRate * 2;
    } else if (distance1 < SafeDistance) {
      // If the left sensor detects an obstacle, move right
      Setpoint = map(closestDistance, 0, SafeDistance, 90, 30) + distanceChangeRate * 2;
    } else if (distance3 < SafeDistance) {
      // If the right sensor detects an obstacle, move left
      Setpoint = map(closestDistance, 0, SafeDistance, -90, -30) - distanceChangeRate * 2;
    } else {
      Setpoint = 0;
    }
  } else {
    // No obstacles detected within SafeDistance, maintain course
    Setpoint = 0;
  }
}

bool checkIrSensors() {
  int irValue1 = analogRead(IrSensor1Pin);
  int irValue2 = analogRead(IrSensor2Pin);
  
  if (irValue1 < IrThreshold || irValue2 < IrThreshold) {
    // Drop detected
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

void reverseAndTurn() {
  // Reverse the motors for a short time
  setMotorSpeed(-255, -255);
  delay(1000);
  
  // Turn the robot
  setMotorSpeed(-255, 255);
  delay(1000);
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
