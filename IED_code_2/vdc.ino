

void obstacleAvoidance(long* distances) {
  // Returns a boolean that determines if safeDistance has been breached 
  if (checkDist(distances, SafeDistance)) {
    /*
    Serial.println();
    Serial.println("SAFE DIST BREACH ---------------------------------------------------------");
    */
    changeDirection(false); // Random direction change without a drop
    
    //Serial.println("SAFE DIST RESET ----------------------------------------------------------");
    
  }
}

void dropAvoidance() {
  dropFlag = 1;
}

void forceWait(int margin) {
  // Debugging info
  Serial.println();
  Serial.println("FORCE-WAIT -------------------------------------------------------------");

  // Force wait till adjustment outputs are really small
  while (!stoppedL || !stoppedR) {
    Serial.println("FORCE WAIT LOOP ... ... ... ... ...");
    /* PID ------------------------------------------------------------------ */
    //long* data = readAllDistances();
    //obstacleAvoidance(data); // Constantly checks for the need for direction change

    InputL = encoderLCount;
    InputR = encoderRCount;

    if (!stoppedL){
      //Serial.println("PID-LEFT");
      myPIDLeft.Compute();
      leftPWM = normalizeToPWM(maxTicksPerSec, OutputL);  // Fixed to use OutputL
      setMotorSpeedL(leftPWM);
    }
    if (!stoppedR){
      //Serial.println("PID-RIGHT");
      myPIDRight.Compute();
      rightPWM = normalizeToPWM(maxTicksPerSec, OutputR); // Fixed to use OutputR
      setMotorSpeedR(rightPWM);
    }
    /* PID ------------------------------------------------------------------ */

      // Debugging info 
    /* 
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
    */

    stopPoint(OutputL, OutputR);
    
    /*
    Serial.print("StopLFlag: ");
    Serial.print(stoppedL);
    Serial.print(" StopRFlag: ");
    Serial.println(stoppedR);
    Serial.println();
    */
  }

  // Debugging info
  //Serial.println("FORCE-WAIT COMPLETED-------------------------------------------------------------");
}

void drive(int desiredDist) {
  /*
  Serial.print("Driving forwards or backwards by ");
  Serial.print(desiredDist);
  Serial.println(" cm.");
  */
  stoppedL = 0;
  stoppedR = 0;
  // Set encoders back to 0 for PID
  encoderLCount = 0;
  encoderRCount = 0;

  // Forward set distance
  int driveDist = distanceToWheelRev(desiredDist, wheelDiameter, ticksPerRev);

  SetpointL = driveDist;
  SetpointR = driveDist;
  
  // Debugging info
  /*
  Serial.print("Drive SetpointL: ");
  Serial.print(SetpointL);
  Serial.print(" SetpointR: ");
  Serial.println(SetpointR);
  */
}

void changeDirection(bool forced) {
  //Serial.println("Change Direction Loop Activated");

  // Forced 180 direction change (in cases where there is a drop)
  int deg = 180;
  int ticks = turnAngleToWheelRev(deg, driveBase, wheelDiameter, ticksPerRev);

  // Otherwise random direction change
  if (!forced) {
    deg = generateRandomValue();
    ticks = turnAngleToWheelRev(deg, driveBase, wheelDiameter, ticksPerRev);
  }

  // Debugging info
  /*
  Serial.print("Turning angle degrees: ");
  Serial.println(deg);
  */
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
  /*
  Serial.print("Change Direction SetpointL: ");
  Serial.print(SetpointL);
  Serial.print(" SetpointR: ");
  Serial.println(SetpointR);
  
  Serial.print("StopLFlag: ");
  Serial.print(stoppedL);
  Serial.print(" StopRFlag: ");
  Serial.println(stoppedR);
  Serial.println();
  */
  forceWait(5);

  // End alert sound after direction change finished
  noTone(SpeakerPin);
  // Debugging info
  //Serial.println("Direction change completed.");
}

void setMotorSpeedL(int motorLSpeed) {
  // Debugging info
  /*
  Serial.print("Setting motor speeds - Left: ");
  Serial.println(motorLSpeed);
  */

  if (motorLSpeed > 0) {
    digitalWrite(MotorLPin1, HIGH);
    digitalWrite(MotorLPin2, LOW);
    digitalWrite(MotorLPin1_dup, HIGH);
    digitalWrite(MotorLPin2_dup, LOW);
    reverseL = 0;
  } else {
    digitalWrite(MotorLPin1, LOW);
    digitalWrite(MotorLPin2, HIGH);
    digitalWrite(MotorLPin1_dup, LOW);
    digitalWrite(MotorLPin2_dup, HIGH);
    motorLSpeed = -motorLSpeed;
    reverseL = 1;
  }
  analogWrite(MotorLSpeedPin, motorLSpeed);
  analogWrite(MotorLSpeedPin_dup, motorLSpeed);
  
}

void setMotorSpeedR(int motorRSpeed) {
  // Debugging info
  /*
  Serial.print("Setting motor speeds - Right: ");
  Serial.println(motorRSpeed);
  */
  
  if (motorRSpeed > 0) {
    digitalWrite(MotorRPin1, HIGH);
    digitalWrite(MotorRPin2, LOW);
    digitalWrite(MotorRPin1_dup, HIGH);
    digitalWrite(MotorRPin2_dup, LOW);
    reverseR = 0;
  } else {
    digitalWrite(MotorRPin1, LOW);
    digitalWrite(MotorRPin2, HIGH);
    digitalWrite(MotorRPin1_dup, LOW);
    digitalWrite(MotorRPin2_dup, HIGH);
    motorRSpeed = -motorRSpeed;
    reverseR = 1;
  }
  analogWrite(MotorRSpeedPin, motorRSpeed);
  analogWrite(MotorRSpeedPin_dup, motorRSpeed);
}

void stopPoint(float setpL, float setpR) {
  // Debugging info
  /*
  Serial.print("Output - Left: ");
  Serial.print(setpL);
  Serial.print(", Right: ");
  Serial.println(setpR);
  */

  if (abs(setpL) < stopMargin) {
    Serial.println("STOP-LEFT");
    analogWrite(MotorLSpeedPin, 0);
    analogWrite(MotorLSpeedPin_dup, 0);
    encoderLCount = 0;
    SetpointL = 0;
    stoppedL = 1;
  }

  if (abs(setpR) < stopMargin) {
    Serial.println("STOP-RIGHT");
    analogWrite(MotorRSpeedPin, 0);
    analogWrite(MotorRSpeedPin_dup, 0);
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