
#include "ZippyCommand.h"
#include "Lighthouse.h"
#include "MotorDriver.h"

//the minimum PCM value below which the motors do not turn
#define MOTOR_MIN_POWER                      3000.00d
#define LINEAR_VELOCITY_POWER               20000.00d
#define HALF_TURNING_RADIUS                   M_PI_2
//#define HALF_TURNING_RADIUS                   1.963495408493621d
//#define HALF_TURNING_RADIUS                   1.178097245096172
#define HALF_SENSOR_SEPARATION 11.0d

//the radius squared (to prevent the need for an additional square root) when we are can consider the robot to be "at the target"
//currently set to 4cm, since sqrt(1600mm)/(10mm per cm) = 4cm; will continue to drive this down as the self-driving capability gets
//more sophisticated and accurate
#define AUTODRIVE_POSITION_EPSILON_2         1600.0d

//checkpoint
//#define MAX_LINEAR_VELOCITY                   600.00d
//#define MAX_ROTATIONAL_VELOCITY               180.00d
//#define LOOK_AHEAD_DISTANCE_MM                250.00d
//#define AUTODRIVE_LINEAR_Kp                    20.20d
//#define AUTODRIVE_LINEAR_Ki                     5.80d
//#define AUTODRIVE_LINEAR_Kd                     1.30d
//checkpoint #2
//#define MAX_LINEAR_VELOCITY                   600.00d
//#define MAX_ROTATIONAL_VELOCITY               250.00d
//#define LOOK_AHEAD_DISTANCE_MM                250.00d
//#define AUTODRIVE_LINEAR_Kp                    19.00d
//#define AUTODRIVE_LINEAR_Ki                     6.20d
//#define AUTODRIVE_LINEAR_Kd                     1.40d
//experimenting
#define MAX_LINEAR_VELOCITY                   800.00d
#define MAX_ROTATIONAL_VELOCITY               260.00d
#define LOOK_AHEAD_DISTANCE_MM                250.00d
#define AUTODRIVE_LINEAR_Kp                    19.15d
#define AUTODRIVE_LINEAR_Ki                     6.55d
#define AUTODRIVE_LINEAR_Kd                     1.46d
#define MAX_SETPOINT_CHANGE                   180.00d

extern Lighthouse lighthouse;
extern MotorDriver motors;

Pause::Pause(double seconds)
  : deltaTimeMS(seconds * 1000.0d),
    startTimeMS(0)
{
}

void Pause::start()
{
  motors.setMotors(0, 0);
  startTimeMS = millis();
}

bool Pause::loop()
{
  return (millis() - startTimeMS) >= deltaTimeMS;
}

double padInner(double motorPower, double magnitude)
{
  if (motorPower > 0.0d)
    return magnitude + motorPower;
  else if (motorPower < 0.0d)
    return -magnitude + motorPower;

  return 0.0d;
}

FollowPath::FollowPath(KVector2* pathPoints,
                       int pathPointCount,
                       int sampleTime)
  : pathPoints(pathPoints),
    pathPointCount(pathPointCount),
    leftPID(&leftInput, &leftOutput, &leftSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, P_ON_E, DIRECT),
    rightPID(&rightInput, &rightOutput, &rightSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, P_ON_E, DIRECT)
{
  leftPID.SetSampleTime(sampleTime);
  leftPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
  rightPID.SetSampleTime(sampleTime);
  rightPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
}

/* leaving this code in, but commented out for now, because I may switch back to this type of mechanism in the near future
//deltaToNextPosition is the next desired position, relative to the current center and orientation of the robot
double FollowPath::calculateInput(LighthouseSensor* sensor,
                                       KVector2* deltaToNextPosition)
{
  //step 1; obtain the sensor position in the global coordinate system
  KVector2* sensorPosition = sensor->getPosition();

  //now translate it to the coordinate system of the robot
  KVector2* robotCenterPosition = lighthouse.getPosition();
  KVector2 sensorPositionFromCenter(sensorPosition->getX() - robotCenterPosition->getX(),
                                    sensorPosition->getY() - robotCenterPosition->getY());
  sensorPositionFromCenter.rotate(-lighthouse.getOrientation()->getOrientation());
  
  //step 2; calculate the vector from current sensor position to desired sensor position  
  //start with the relative sensor position
  KVector2 desiredSensorPosition(&sensorPositionFromCenter);
//  desiredSensorPosition.printDebug();

  //rotate it by the amount of rotation from our current orientation to the desired position
  desiredSensorPosition.rotate(deltaToNextPosition->getOrientation());

  //add the it to the next desired position of the center of the robot
  desiredSensorPosition.addVector(deltaToNextPosition);
//  desiredSensorPosition.printDebug();

  //subtract the current relative sensor position from the center of the robot to obtain the vector from the current sensor
  //position to the desired sensor position
  desiredSensorPosition.subtractVector(&sensorPositionFromCenter);
//  desiredSensorPosition.printDebug();
//  SerialUSB.println();

  //now get the angle from the sensor vector to the vector from the sensor to the desired sensor position as a range from
  //+1.0 to -1.0; this gives us the mix of linear velocity and the rotational velocity
  double linearVelocityRatio = desiredSensorPosition.angleToVector(lighthouse.getOrientation()) / M_PI;
  linearVelocityRatio = (linearVelocityRatio < 0.0d ? 1.0d + linearVelocityRatio : -1.0d + linearVelocityRatio);
//  sensorPositionFromCenter.set(-sensorPositionFromCenter.getX(), -sensorPositionFromCenter.getY());
  double rotationalVelocityRatio = desiredSensorPosition.angleToVector(&sensorPositionFromCenter) / M_PI;
  return (MAX_LINEAR_VELOCITY * linearVelocityRatio) + (MAX_ROTATIONAL_VELOCITY * rotationalVelocityRatio);
}
*/

void FollowPath::start()
{
  leftPID.SetMode(MANUAL);
  rightPID.SetMode(MANUAL);

  //capture our starting position
  firstPosition.set(lighthouse.getPosition());

  //setup the current segment
  currentSegmentStart = &firstPosition;
  currentSegmentEndIndex = 0;
  currentSegment.set(pathPoints[currentSegmentEndIndex].getX() - currentSegmentStart->getX(),
      pathPoints[currentSegmentEndIndex].getY() - currentSegmentStart->getY());
  currentDistanceAlongSegment = 0.0d;
  
  //reset the set points
  updateInputs();

  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source for details
  leftOutput = 0.0d;
  rightOutput = 0.0d;

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  motors.setMotors(padInner(-leftOutput, MOTOR_MIN_POWER),
                   padInner(-rightOutput, MOTOR_MIN_POWER));
}

bool FollowPath::loop()
{
  if (currentSegmentEndIndex+1 == pathPointCount) {
    //we're on the last path segment, so check to see if we've reached the last endpoint on that segment
    KVector2* currentPosition = lighthouse.getPosition();
    KVector2 deltaCenterToTarget(pathPoints[currentSegmentEndIndex].getX() - currentPosition->getX(),
        pathPoints[currentSegmentEndIndex].getY() - currentPosition->getY());
    if (deltaCenterToTarget.getD2() < AUTODRIVE_POSITION_EPSILON_2) {
      leftPID.SetMode(MANUAL);
      rightPID.SetMode(MANUAL);
      return true;
    }
  }

  updateInputs();

  leftPID.Compute();
  rightPID.Compute();

//  SerialUSB.print("L: ");
//  SerialUSB.print(leftOutput, 2);
//  SerialUSB.print("   R: ");
//  SerialUSB.println(rightOutput, 2);

  motors.setMotors(padInner(-leftOutput, MOTOR_MIN_POWER),
                   padInner(-rightOutput, MOTOR_MIN_POWER));

  return false;
}

void FollowPath::updateInputs()
{
  KVector2 nextPosition;
  calculateNextPosition(&nextPosition);

  //now make the reference to the next position relative to the local coordinate system of the robot
  KVector2* lighthouseOrientation = lighthouse.getOrientation();
  nextPosition.rotate(-lighthouseOrientation->getOrientation());
//  nextPosition.printDebug();

  LighthouseSensor* leftSensor = lighthouse.getLeftSensor();
  LighthouseSensor* rightSensor = lighthouse.getRightSensor();
  double angleToPosition = nextPosition.getOrientation();
  if (angleToPosition <= -HALF_TURNING_RADIUS) {
    //target is behind us to the left; max turn left
    leftTargetSetPoint = -MAX_ROTATIONAL_VELOCITY;
    rightTargetSetPoint = MAX_ROTATIONAL_VELOCITY;
  }
  else if (angleToPosition >= HALF_TURNING_RADIUS) {
    //target is behind us to the right; max turn right
    leftTargetSetPoint = MAX_ROTATIONAL_VELOCITY;
    rightTargetSetPoint = -MAX_ROTATIONAL_VELOCITY;
  }
  else {
    //normalize the angle to be in the range of -1.0 to +1.0
    angleToPosition /= HALF_TURNING_RADIUS;

    //use the normalized angle as the ratio of moving forward versus turning
    double linearVelocity = MAX_LINEAR_VELOCITY * (1.0d - abs(angleToPosition));
    double rotationalVelocity = MAX_ROTATIONAL_VELOCITY * angleToPosition;
    leftTargetSetPoint = linearVelocity + rotationalVelocity;
    rightTargetSetPoint = linearVelocity - rotationalVelocity;
  }

//  /*
  double leftSetPointDelta = leftTargetSetPoint - leftSetPoint;
  if (leftSetPointDelta >= 0.0)
    leftSetPointDelta = min(leftSetPointDelta, MAX_SETPOINT_CHANGE);
  else
    leftSetPointDelta = max(leftSetPointDelta, -MAX_SETPOINT_CHANGE);
  leftSetPoint += leftSetPointDelta;

  double rightSetPointDelta = rightTargetSetPoint - rightSetPoint;
  if (rightSetPointDelta >= 0.0)
    rightSetPointDelta = min(rightSetPointDelta, MAX_SETPOINT_CHANGE);
  else
    rightSetPointDelta = max(rightSetPointDelta, -MAX_SETPOINT_CHANGE);
  rightSetPoint += rightSetPointDelta;
//  */
//  leftSetPoint = leftTargetSetPoint;
//  rightSetPoint = rightTargetSetPoint;

  leftInput = leftSensor->getVelocity();
  rightInput = rightSensor->getVelocity();
}

void FollowPath::calculateNextPosition(KVector2* nextPosition)
{
  //determine the current target position relative to the robot
  getCurrentTargetPosition(nextPosition);
  
  //check if we're within the look-ahead distance of the current target position
  double distanceToNextPosition = nextPosition->getD();
  if (distanceToNextPosition >= LOOK_AHEAD_DISTANCE_MM) {
    //still moving to next position; just keep moving toward it
    nextPosition->setD(LOOK_AHEAD_DISTANCE_MM);
    return;
  }
  
  //the robot is within the look-ahead distance of the current target position, so move the target position forward along the path by the difference
  currentDistanceAlongSegment += (LOOK_AHEAD_DISTANCE_MM - distanceToNextPosition);

  //if we've reached beyond the end of the current path segment and there are more segments, move to the next segment
  while (currentDistanceAlongSegment >= currentSegment.getD()) {
    //stop at the last segment
    if (currentSegmentEndIndex+1 == pathPointCount) {
      currentDistanceAlongSegment = currentSegment.getD();
      break;
    }
    
    //there are more path segments, so we're going to move on to the next path segment
    //first determine the distance left over after completing the current segment
    currentDistanceAlongSegment -= currentSegment.getD();

    //move to the next path segment
    currentSegmentStart = &pathPoints[currentSegmentEndIndex];
    currentSegmentEndIndex++;
    currentSegment.set(pathPoints[currentSegmentEndIndex].getX() - currentSegmentStart->getX(),
        pathPoints[currentSegmentEndIndex].getY() - currentSegmentStart->getY());
  }

  //recalculate our current target position along the path
  getCurrentTargetPosition(nextPosition);
  nextPosition->setD(min(LOOK_AHEAD_DISTANCE_MM, nextPosition->getD()));
}

//calculate our current target position along the current path segment, relative to the current position of the robot
void FollowPath::getCurrentTargetPosition(KVector2* nextPosition)
{
  //start by determining the current distance we've traveled along this segment, normalized from 0.0 to 1.0
  double deltaAlongPath = currentDistanceAlongSegment / currentSegment.getD();
  
  //now LERP out the x/y position along the current path segment
  nextPosition->set(currentSegmentStart->getX() + (currentSegment.getX() * deltaAlongPath),
      currentSegmentStart->getY() + (currentSegment.getY() * deltaAlongPath));
      
  //subtract the current position of the robot to obtain a vector relative to the robot to the target position
  nextPosition->subtractVector(lighthouse.getPosition());
}

