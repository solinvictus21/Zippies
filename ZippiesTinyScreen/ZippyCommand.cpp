
#include "ZippyCommand.h"
#include "Lighthouse.h"
#include "MotorDriver.h"

//the minimum PCM value below which the motors do not turn
//#define MOTOR_MIN_POWER                      20000.00d
#define LINEAR_VELOCITY_POWER               50000.00d
//#define HALF_TURNING_RADIUS                   M_PI_2
// 3/8 * M_PI
#define HALF_TURNING_RADIUS                   1.178097245096172d
#define HALF_SENSOR_SEPARATION 11.0d

//the radius squared (to prevent the need for an additional square root) when we are can consider the robot to be "at the target"
//currently set to 4cm, since sqrt(1600mm)/(10mm per cm) = 4cm; will continue to drive this down as the self-driving capability gets
//more sophisticated and accurate
#define AUTODRIVE_POSITION_EPSILON_2         2500.0d
//#define AUTODRIVE_POSITION_EPSILON_2         1600.0d

/*
//fast
#define MOTOR_MIN_POWER                      3000.00d
#define MAX_LINEAR_VELOCITY                  1000.00d
#define MAX_ROTATIONAL_VELOCITY               250.00d
#define LOOK_AHEAD_DISTANCE_MM                280.00d
//when the robot reaches within the LOOK_AHEAD_DISTANCE, the next point to look toward along the path is incremented by this distance
#define LOOK_AHEAD_INCREMENTS_MM               20.00d
//the right can be more aggressive than the left because acceleration on the left offset wheel can cause the right wheel to pull up off the ground
#define LEFT_Kp                                19.00d
#define LEFT_Ki                                 1.20d
#define LEFT_Kd                                 3.70d
#define RIGHT_Kp                               21.00d
#define RIGHT_Ki                                1.20d
#define RIGHT_Kd                                2.90d
#define MAX_SETPOINT_CHANGE                   150.00d
*/

///*
//slow
#define MOTOR_MIN_POWER                      3000.00d
#define MAX_LINEAR_VELOCITY                  1000.00d
#define MAX_ROTATIONAL_VELOCITY               250.00d
#define LOOK_AHEAD_DISTANCE_MM                280.00d
//when the robot reaches within the LOOK_AHEAD_DISTANCE, the next point to look toward along the path is incremented by this distance
#define LOOK_AHEAD_INCREMENTS_MM               20.00d
//the right can be more aggressive than the left because acceleration on the left offset wheel can cause the right wheel to pull up off the ground
#define LEFT_Kp                                13.50d
#define LEFT_Ki                                 0.00d
#define LEFT_Kd                                 2.30d
#define RIGHT_Kp                               16.00d
#define RIGHT_Ki                                0.00d
#define RIGHT_Kd                                0.40d
#define MAX_SETPOINT_CHANGE                   180.00d
//*/

extern Lighthouse lighthouse;
extern MotorDriver motors;

KVector2 leftWheelOffset( -6.9d, -1.7d);
KVector2 rightWheelOffset(+6.9d, +10.1d);

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
    leftPID(&leftInput, &leftOutput, &leftSetPoint, LEFT_Kp, LEFT_Ki, LEFT_Kd, P_ON_E, DIRECT),
    rightPID(&rightInput, &rightOutput, &rightSetPoint, RIGHT_Kp, RIGHT_Ki, RIGHT_Kd, P_ON_E, DIRECT)
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
  
  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  leftSetPoint = 0.0d;
  rightSetPoint = 0.0d;
  leftInput = 0.0d;
  rightInput = 0.0d;
  leftOutput = 0.0d;
  rightOutput = 0.0d;

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  //reset the set points
//  updateInputs();

//  leftPID.Compute();
//  rightPID.Compute();

//  motors.setMotors(padInner(-leftOutput, MOTOR_MIN_POWER),
//                   padInner(-rightOutput, MOTOR_MIN_POWER));
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

  motors.setMotors(padInner(-leftOutput, MOTOR_MIN_POWER),
                   padInner(-rightOutput, MOTOR_MIN_POWER));

//  motors.setMotors(-leftOutput, -rightOutput);

  return false;
}

//#define TRUNCATE_LINEAR_ANGLE 0.3d

void FollowPath::updateInputs()
{
  KVector2 nextPosition;
  calculateNextPosition(&nextPosition);

  //now make the reference to the next position relative to the local coordinate system of the robot
  KVector2* lighthouseOrientation = lighthouse.getOrientation();
  nextPosition.rotate(-lighthouseOrientation->getOrientation());
//  nextPosition.printDebug();

  LighthouseSensor* leftSensor = lighthouse.getLeftSensor();
//  leftSensor->addVector(&leftWheelOffset);
  LighthouseSensor* rightSensor = lighthouse.getRightSensor();
//  rightSensor->addVector(&rightWheelOffset);
  double angleToPosition = nextPosition.getOrientation();
  double leftTargetSetPoint, rightTargetSetPoint;
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
//    double upperLimit = 1.0d - TRUNCATE_LINEAR_ANGLE;
//    double linearVelocity = MAX_LINEAR_VELOCITY * (1.0d - (max(0.0d, min(upperLimit, abs(angleToPosition)) - TRUNCATE_LINEAR_ANGLE) / (upperLimit - TRUNCATE_LINEAR_ANGLE)));
    double linearVelocity = MAX_LINEAR_VELOCITY * (1.0d - max(0.0d, abs(angleToPosition)-0.3d));
    double rotationalVelocity = MAX_ROTATIONAL_VELOCITY * angleToPosition;
    leftTargetSetPoint = linearVelocity + rotationalVelocity;
    rightTargetSetPoint = linearVelocity - rotationalVelocity;
  }

///*
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
//*/

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
  while (distanceToNextPosition < LOOK_AHEAD_DISTANCE_MM &&
      //stop at the end of the path
      (currentDistanceAlongSegment < currentSegment.getD() || currentSegmentEndIndex+1 < pathPointCount))
  {
    //the robot is within the look-ahead distance of the current target position, so move the target position forward along the path
    currentDistanceAlongSegment += (LOOK_AHEAD_DISTANCE_MM - distanceToNextPosition) + LOOK_AHEAD_INCREMENTS_MM;
  
    while (currentDistanceAlongSegment > currentSegment.getD()) {
      //the look-ahead distance is beyond the end of the current path segment
      if (currentSegmentEndIndex+1 == pathPointCount) {
        //but this is the last segment, so stop at the end
        currentDistanceAlongSegment = currentSegment.getD();
        break;
      }

      //there are more path segments, so move to the next path segment
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

    //check our distance to the next position
    distanceToNextPosition = nextPosition->getD();
  }

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

