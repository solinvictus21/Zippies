
#include "FollowPath.h"

//power to each wheel is tuned as a combination of targets for rotational power and linear power; the linear power target defines the motion forward
//or backward we wish the robot to drive and will be the same for both wheels; the rotational power target defines the offest from linear power that
//we wish to apply to force the robot to turn and will be equal magnitude but opposite direction for each wheel

//TUNING - PATH PLANNING
#define LOOP_INTERVAL_MS                         40

//TUNING - PID CONFIGURATION
#define LINEAR_Kp                                19.00d
#define LINEAR_Ki                                 0.00d
#define LINEAR_Kd                                 0.20d
#define ROTATIONAL_Kp                             8.40d
#define ROTATIONAL_Ki                             0.00d
#define ROTATIONAL_Kd                             0.42d

//TUNING - PID INPUT
//70 degrees (70/180 * M_PI)
// #define LINEAR_RAMPUP_START                       1.221730476396031d
//80 degrees
#define LINEAR_RAMPUP_START                       1.396263401595464d
//30 degrees (30/180 * M_PI)
// #define LINEAR_RAMPUP_END                         0.523598775598299d
//40
// #define LINEAR_RAMPUP_END                         0.698131700797732d
//50
#define LINEAR_RAMPUP_END                         0.872664625997165d
#define LINEAR_MAX_VELOCITY                    1000.00d
#define LINEAR_AVG_VELOCITY                     600.00d
#define LINEAR_MIN_INPUT_CHANGE                  30.00d
#define LINEAR_MAX_INPUT_CHANGE_FACTOR            0.10d

//40 degrees
// #define ROTATIONAL_RAMPDOWN_START                 0.698131700797732d
//45
#define ROTATIONAL_RAMPDOWN_START                 0.785398163397448d
//50
// #define ROTATIONAL_RAMPDOWN_START                 0.872664625997165d
//60
// #define ROTATIONAL_RAMPDOWN_START               1.047197551196598d
// 90
// #define ROTATIONAL_RAMPDOWN_START               M_PI_2
#define ROTATIONAL_MIN_VELOCITY                 500.0d

//TUNING - PCM OUTPUT
//the minimum PCM value below which the motors do not turn
#define MOTOR_MIN_POWER                       3200.00d
//the maximum PCM output the PID controllers are limited to produce out of an absolute max of 65535
#define MOTOR_MAX_POWER                      50000.00d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince within which is to be considered "at the target" for the purpose of terminating the current driving command; we use the square
//of the radius to eliminate the need for an additional square root
//sqrt(2500mm)/(10mm per cm) = 5cm
// #define AUTODRIVE_POSITION_EPSILON_2         2500.0d

double padInner(double motorPower, double magnitude)
{
  if (motorPower > 0.0d)
    return magnitude + motorPower;
  else if (motorPower < 0.0d)
    return -magnitude + motorPower;

  return 0.0d;
}

FollowPath::FollowPath(Zippy* z,
                       KVector2** pathPoints,
                       int pathPointCount)
  : zippy(z),
    pathPoints(pathPoints),
    pathPointCount(pathPointCount),
    linearPID(&linearInput, &linearOutput, &linearSetPoint, LINEAR_Kp, LINEAR_Ki, LINEAR_Kd, P_ON_E, DIRECT),
    rotationalPID(&rotationalInput, &rotationalOutput, &rotationalSetPoint, ROTATIONAL_Kp, ROTATIONAL_Ki, ROTATIONAL_Kd, P_ON_E, DIRECT)
{
  linearPID.SetSampleTime(LOOP_INTERVAL_MS);
  linearPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
  rotationalPID.SetSampleTime(LOOP_INTERVAL_MS);
  rotationalPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
  totalPathLength = 0;
  for (int i = 0; i < pathPointCount-1; i++) {
    totalPathLength += sqrt(pow(pathPoints[i+1]->getX() - pathPoints[i]->getX(), 2.0d) +
        pow(pathPoints[i+1]->getY() - pathPoints[i]->getY(), 2.0d));
  }
  targetDeltaTime = (totalPathLength / LINEAR_AVG_VELOCITY) * 1000.0d;
  // SerialUSB.println(targetDeltaTime);
}

void FollowPath::start(unsigned long currentTime)
{
  zippy->recalculate();

  linearPID.SetMode(MANUAL);
  rotationalPID.SetMode(MANUAL);

  //capture our starting position
  firstPosition.set(zippy->getPosition());

  //setup the current segment
  // currentSegmentStart = &firstPosition;
  // currentSegmentEndIndex = 0;
  currentSegmentStart = pathPoints[0];
  currentSegmentEndIndex = 1;
  currentSegment.set(pathPoints[currentSegmentEndIndex]->getX() - currentSegmentStart->getX(),
      pathPoints[currentSegmentEndIndex]->getY() - currentSegmentStart->getY());
  currentDistanceAlongSegment = 0.0d;
  currentDistanceAlongPath = 0.0d;
  pathStartTime = currentTime;

  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  linearSetPoint = 0.0d;
  linearInput = 0.0d;
  linearOutput = 0.0d;
  rotationalSetPoint = 0.0d;
  rotationalInput = 0.0d;
  rotationalOutput = 0.0d;

  linearPID.SetMode(AUTOMATIC);
  rotationalPID.SetMode(AUTOMATIC);

  lastUpdateTime = currentTime;
}

bool FollowPath::loop(unsigned long currentTime)
{
  if (currentTime - pathStartTime >= targetDeltaTime) {
    //our time has run out
    return true;
  }

  if (currentTime - lastUpdateTime < LOOP_INTERVAL_MS) {
    return false;
  }

  zippy->recalculate();

  updateInputs(currentTime);

  linearPID.Compute();
  rotationalPID.Compute();

  zippy->setMotors(padInner(-linearOutput-rotationalOutput, MOTOR_MIN_POWER),
                   padInner(-linearOutput+rotationalOutput, MOTOR_MIN_POWER));
  // zippy->setMotors(padInner(-rotationalOutput, MOTOR_MIN_POWER),
                   // padInner(+rotationalOutput, MOTOR_MIN_POWER));

 lastUpdateTime += LOOP_INTERVAL_MS;

  return false;
}

void FollowPath::updateInputs(unsigned long currentTime)
{
  if (currentDistanceAlongPath < totalPathLength) {
    //based on the time we've been on this path, calculate the distance that we should currently be through the entire path
    double newDistanceAlongPath = (((double)(currentTime - pathStartTime + LOOP_INTERVAL_MS)) / ((double)targetDeltaTime)) * totalPathLength;
    double deltaDistance = newDistanceAlongPath - currentDistanceAlongPath;
    currentDistanceAlongSegment += deltaDistance;
    currentDistanceAlongPath = newDistanceAlongPath;
    // SerialUSB.println(((double)(currentTime - lastUpdateTime)), 2);
  }

  //determine our next target position
  KVector2 nextPosition;
  getCurrentTargetPosition(&nextPosition);

  //subtract the current position and orientation of the robot to obtain a vector relative to the robot to the target position
  nextPosition.subtractVector(zippy->getPosition());
  nextPosition.rotate(-zippy->getOrientation()->getOrientation());

  //TODO: slow down as we approach the last point on the path
  // double targetLinearVelocity = LINEAR_MAX_VELOCITY * (1.0d - constrain((abs(angleToPosition) - LINEAR_RAMPUP_END) / (LINEAR_RAMPUP_START - LINEAR_RAMPUP_END), 0.0d, 1.0d));
  double angleToPosition = nextPosition.getOrientation();
  double targetLinearVelocity = min((nextPosition.getD() * 1000.0d) / ((double)LOOP_INTERVAL_MS), LINEAR_MAX_VELOCITY) *
      (1.0d - constrain((abs(angleToPosition) - LINEAR_RAMPUP_END) / (LINEAR_RAMPUP_START - LINEAR_RAMPUP_END), 0.0d, 1.0d));
  double maxInputChange = max(abs(linearInput) * LINEAR_MAX_INPUT_CHANGE_FACTOR, LINEAR_MIN_INPUT_CHANGE);
  KVector2* currentVelocityVector = zippy->getVelocity();
  double currentVelocity = currentVelocityVector->getD();
  if (currentVelocityVector->dotVector(zippy->getOrientation()) < 0.0d)
    currentVelocity = -currentVelocity;
  linearInput += constrain(currentVelocity - targetLinearVelocity - linearInput, -maxInputChange, maxInputChange);
  // linearInput = currentVelocity - targetLinearVelocity;

  // rotationalInput = (abs(angleToPosition) < ROTATIONAL_RAMPDOWN_START ? tan(angleToPosition) : (angleToPosition < 0.0d ? 1.0d : -1.0d)) *
  rotationalInput = constrain(-angleToPosition / ROTATIONAL_RAMPDOWN_START, -1.0d, 1.0d) *
      ROTATIONAL_MIN_VELOCITY;
      // max(ROTATIONAL_MIN_VELOCITY, abs(currentVelocity));
}

/*
void FollowPath::updateInputs(unsigned long currentTime)
{
  //determine our next target position
  KVector2 nextPosition;
  calculateNextPosition(&nextPosition);

  double angleToPosition = nextPosition.getOrientation();
  KVector2* currentVelocityVector = zippy->getVelocity();

  //TODO: slow down as we approach the last point on the path
  double targetLinearVelocity = LINEAR_MAX_VELOCITY * (1.0d - constrain((abs(angleToPosition) - LINEAR_RAMPUP_END) / (LINEAR_RAMPUP_START - LINEAR_RAMPUP_END), 0.0d, 1.0d));
  double maxInputChange = max(abs(linearInput) * LINEAR_MAX_INPUT_CHANGE_FACTOR, LINEAR_MIN_INPUT_CHANGE);
  double currentVelocity = currentVelocityVector->getD();
  if (currentVelocityVector->dotVector(zippy->getOrientation()) < 0.0d)
    currentVelocity = -currentVelocity;
  linearInput += constrain(currentVelocity - targetLinearVelocity - linearInput, -maxInputChange, maxInputChange);

  rotationalInput = constrain(-angleToPosition / ROTATIONAL_RAMPDOWN_START, -1.0d, 1.0d) * max(400.0d, abs(currentVelocity));//1000.0d;
  // rotationalInput = constrain(-angleToPosition / ROTATIONAL_RAMPDOWN_START, -1.0d, 1.0d) * max(500.0d, targetLinearVelocity);//1000.0d;
}

void FollowPath::calculateNextPosition(KVector2* nextPosition)
{
  //determine the current target position relative to the robot
  getCurrentTargetPosition(nextPosition);

  //check if we're within the look-ahead distance of the current target position
  double distanceToNextPosition = nextPosition->getD();
  while (distanceToNextPosition < LOOK_AHEAD_DISTANCE_MM &&
      //stop at the end of the path
      (currentDistanceAlongSegment < currentSegment.getD() || currentSegmentEndIndex+1 < pathPointCount))
  {
    //the robot is within the look-ahead distance of the current target position, so move the target position forward along the path
    currentDistanceAlongSegment += (LOOK_AHEAD_DISTANCE_MM - distanceToNextPosition) + LOOK_AHEAD_INCREMENTS_MM;

    //recalculate our current target position along the path
    getCurrentTargetPosition(nextPosition);

    //subtract the current position of the robot to obtain a vector relative to the robot to the target position
    nextPosition->subtractVector(zippy->getPosition());

    //check our distance to the next position
    distanceToNextPosition = nextPosition->getD();
  }

  //now make the reference to the next position relative to the local coordinate system of the robot
  nextPosition->rotate(-zippy->getOrientation()->getOrientation());
  nextPosition->setD(min(LOOK_AHEAD_DISTANCE_MM, nextPosition->getD()));
}
*/

void FollowPath::getCurrentTargetPosition(KVector2* nextPosition)
{
  while (currentDistanceAlongSegment > currentSegment.getD()) {
    //the look-ahead distance is beyond the end of the current path segment
    if (currentSegmentEndIndex+1 == pathPointCount) {
      //but this is the last segment, so stop at the end
      currentDistanceAlongSegment = currentSegment.getD();
      currentDistanceAlongPath = totalPathLength;
      break;
    }

    //there are more path segments, so move to the next path segment
    //first determine the distance left over after completing the current segment
    currentDistanceAlongSegment -= currentSegment.getD();

    //move to the next path segment
    currentSegmentStart = pathPoints[currentSegmentEndIndex];
    currentSegmentEndIndex++;
    currentSegment.set(pathPoints[currentSegmentEndIndex]->getX() - currentSegmentStart->getX(),
        pathPoints[currentSegmentEndIndex]->getY() - currentSegmentStart->getY());
  }

  //start by determining the current distance we've traveled along this segment, normalized from 0.0 to 1.0
  double deltaAlongPath = currentDistanceAlongSegment / currentSegment.getD();

  //now LERP out the x/y position along the current path segment
  nextPosition->set(currentSegmentStart->getX() + (currentSegment.getX() * deltaAlongPath),
      currentSegmentStart->getY() + (currentSegment.getY() * deltaAlongPath));
}
