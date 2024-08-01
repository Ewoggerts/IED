
const float pi = 3.1415926535;


// Converts input turn angle to the required number of wheel rotations to achieve that angle
float turnAngleToWheelRev(float turnAngle, float driveBase, float wheelDiameter, int ticksPerRev){
  float turnCircumference = driveBase * pi;
  float distPerWheelRev = wheelDiameter * pi;
  float wheelRevPerFullRotation = turnCircumference / distPerWheelRev;
  float degreeChangePerWheelRev = wheelRevPerFullRotation / 360;
  float outputTicks = turnAngle * degreeChangePerWheelRev * ticksPerRev;
  return outputTicks;
}

// Converts input travel distance to the required number of wheel rotations to achieve that distance
float distanceToWheelRev(float travelDistance, float wheelDiameter, int ticksPerRev){
  float travelDistancePerWheelRev = wheelDiameter * pi;
  float outputTicks = travelDistance / travelDistancePerWheelRev *ticksPerRev;
  return outputTicks;
}

// Normalizes an input(Ticks per Second) to PWM output using maximum tested encoder ticks per second
float normalizeToPWM(float maxTicksPerSec, float inputTicksPerSec) {
  float normalizedPWMVal = map(inputTicksPerSec, -maxTicksPerSec, maxTicksPerSec, -255, 255);
  return normalizedPWMVal;
}

int generateRandomValue() {
  // Generate a random boolean to decide the range
  bool isNegative = random(0, 2);

  if (isNegative) {
    // Return a random value between -60 to -180
    return random(-180, -59);
  } else {
    // Return a random value between 60 to 180
    return random(60, 181);
  }
}

boolean checkDist(long* distances, int SafeDistance){
  //check if direction change is required
  for(unsigned int i = 0; i < 3; i++){
    if (distances[i] < SafeDistance){
      return true;
    }
  }
  return false;
}

long getDistance(int trigPin, int echoPin) {
  // Send a pulse to trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(echoPin, HIGH);

  // Convert the duration to distance in cm
  long distance = duration * 0.034 / 2;

  return distance;
}