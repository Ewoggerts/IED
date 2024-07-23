#include <Math.h>


float PI(){
  float pi = 3.1415926535;
  return pi;
}


// Converts input turn angle to the required number of wheel rotations to achieve that angle
float turnAngleToWheelRev(float turnAngle, float driveBase, float wheelDiameter, int ticksPerRev){
  float turnCircumference = driveBase * PI();
  float distPerWheelRev = wheelDiameter * PI();
  float wheelRevPerFullRotation = turnCircumference / distPerWheelRev;
  float degreeChangePerWheelRev = wheelRevPerFullRotation / 360;
  float outputRevs = turnAngle * degreeChangePerWheelRev * ticksPerRev;
  return outputTicks;
}

// Converts input travel distance to the required number of wheel rotations to achieve that distance
float distanceToWheelRev(float travelDistance, float wheelDiameter, int ticksPerRev){
  float travelDistancePerWheelRev = wheelDiameter * PI();
  float outputRevs = travelDistance / travelDistancePerWheelRev *ticksPerRev;
  return outputTicks;
}

