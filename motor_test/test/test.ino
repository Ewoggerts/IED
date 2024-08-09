// Define motor pins for first set
const int MotorLPin1 = 22;
const int MotorLPin2 = 23;
const int MotorLSpeedPin = 44;
const int MotorRPin1 = 24;
const int MotorRPin2 = 25;
const int MotorRSpeedPin = 45;

// Define motor pins for second set
const int MotorLPin1_2 = 36;
const int MotorLPin2_2 = 37;
const int MotorLSpeedPin_2 = 46;
const int MotorRPin1_2 = 38;
const int MotorRPin2_2 = 39;
const int MotorRSpeedPin_2 = 4;

void setup() {
  // Initialize motor pins as outputs
  pinMode(MotorLPin1, OUTPUT);
  pinMode(MotorLPin2, OUTPUT);
  pinMode(MotorLSpeedPin, OUTPUT);
  pinMode(MotorRPin1, OUTPUT);
  pinMode(MotorRPin2, OUTPUT);
  pinMode(MotorRSpeedPin, OUTPUT);

  pinMode(MotorLPin1_2, OUTPUT);
  pinMode(MotorLPin2_2, OUTPUT);
  pinMode(MotorLSpeedPin_2, OUTPUT);
  pinMode(MotorRPin1_2, OUTPUT);
  pinMode(MotorRPin2_2, OUTPUT);
  pinMode(MotorRSpeedPin_2, OUTPUT);

  // Set initial state of motor pins to forward direction
  digitalWrite(MotorLPin1, HIGH);
  digitalWrite(MotorLPin2, LOW);
  digitalWrite(MotorRPin1, HIGH);
  digitalWrite(MotorRPin2, LOW);

  digitalWrite(MotorLPin1_2, HIGH);
  digitalWrite(MotorLPin2_2, LOW);
  digitalWrite(MotorRPin1_2, HIGH);
  digitalWrite(MotorRPin2_2, LOW);

  // Set motor speed to 100 PWM
  analogWrite(MotorLSpeedPin, 255);
  analogWrite(MotorRSpeedPin, 255);
  analogWrite(MotorLSpeedPin_2, 255);
  analogWrite(MotorRSpeedPin_2, 255);
}

void loop() {
  // Keep the motors running
}
