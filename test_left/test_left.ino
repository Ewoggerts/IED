// Define motor pins
const int MotorLPin1 = 22;
const int MotorLPin2 = 23;
const int MotorLSpeedPin = 44;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Define motor pins as outputs
  pinMode(MotorLPin1, OUTPUT);
  pinMode(MotorLPin2, OUTPUT);
  pinMode(MotorLSpeedPin, OUTPUT);
}

void loop() {
  // Test motor by running it forward
  Serial.println("Running motor forward...");
  digitalWrite(MotorLPin1, HIGH);
  digitalWrite(MotorLPin2, LOW);
  analogWrite(MotorLSpeedPin, 255); // Set speed to maximum
  delay(2000); // Run for 2 seconds

  // Stop motor
  Serial.println("Stopping motor...");
  digitalWrite(MotorLPin1, LOW);
  digitalWrite(MotorLPin2, LOW);
  analogWrite(MotorLSpeedPin, 0); // Stop the motor
  delay(1000); // Wait for 1 second

  // Test motor by running it backward
  Serial.println("Running motor backward...");
  digitalWrite(MotorLPin1, LOW);
  digitalWrite(MotorLPin2, HIGH);
  analogWrite(MotorLSpeedPin, 255); // Set speed to maximum
  delay(2000); // Run for 2 seconds

  // Stop motor
  Serial.println("Stopping motor...");
  digitalWrite(MotorLPin1, LOW);
  digitalWrite(MotorLPin2, LOW);
  analogWrite(MotorLSpeedPin, 0); // Stop the motor
  delay(1000); // Wait for 1 second
}
