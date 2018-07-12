
#include "ZippyCommand.h"
#include "Lighthouse.h"
#include "MotorDriver.h"

//power to each wheel is tuned as a combination of targets for rotational power and linear power; the linear power target defines the motion forward
//or backward we wish the robot to drive and will be the same for both wheels; the rotational power target defines the offest from linear power that
//we wish to apply to force the robot to turn and will be equal magnitude but opposite direction for each wheel

//TUNING - PID INITIALIZATION
//the right must be more aggressive than the left because acceleration on the left offset wheel can cause the right wheel to pull up off the ground
/*
#define LEFT_Kp                                11.10d
#define LEFT_Ki                                 0.00d
#define LEFT_Kd                                 1.90d
#define RIGHT_Kp                               12.80d
#define RIGHT_Ki                                0.00d
#define RIGHT_Kd                                0.70d
*/
/*
#define LEFT_Kp                                10.00d
#define LEFT_Ki                                 0.00d
#define LEFT_Kd                                 1.60d
#define RIGHT_Kp                               11.00d
#define RIGHT_Ki                                0.00d
#define RIGHT_Kd                                0.80d
*/
#define LINEAR_Kp                                 0.00d
#define LINEAR_Ki                                 0.00d
#define LINEAR_Kd                                 0.00d
#define ROTATIONAL_Kp                          2200.00d
#define ROTATIONAL_Ki                             0.00d
#define ROTATIONAL_Kd                             0.00d

//TUNING - PATH PLANNING
//the minimum distance to look ahead along the current path segment
#define LOOK_AHEAD_DISTANCE_MM                280.00d
//when the robot reaches within the LOOK_AHEAD_DISTANCE_MM, the next point to look toward along the path is incremented by this distance
#define LOOK_AHEAD_INCREMENTS_MM               20.00d

//TUNING - PID SETPOINT
//60 degrees (60/180 * M_PI)
#define LINEAR_RAMPUP_START               1.047197551196598
//70 degrees (70/180 * M_PI)
//#define LINEAR_RAMPUP_START                     1.221730476396031d
//30 degrees (30/180 * M_PI)
#define LINEAR_RAMPUP_END                       0.523598775598299d
#define LINEAR_MAX_VELOCITY                  1000.00d

//60 degrees (60/180 * M_PI)
#define ROTATIONAL_RAMPDOWN_START               1.047197551196598
//70 degrees (70/180 * M_PI)
//#define ROTATIONAL_RAMPDOWN_START               1.221730476396031d
#define ROTATIONAL_MAX_VELOCITY               280.00d

//#define HALF_SENSOR_SEPARATION                 11.0d
#define MAX_SETPOINT_CHANGE                   180.00d

//TUNING - PCM OUTPUT
//the minimum PCM value below which the motors do not turn
#define MOTOR_MIN_POWER                      3000.00d
//the maximum PCM output the PID controllers are limited to produce out of an absolute max of 65535
#define MOTOR_MAX_POWER                     50000.00d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince within which is to be considered "at the target" for the purpose of terminate the current driving command; we use the radius squared
//to prevent the need for an additional square root
//sqrt(2500mm)/(10mm per cm) = 5cm
#define AUTODRIVE_POSITION_EPSILON_2         2500.0d
//sqrt(1600mm)/(10mm per cm) = 4cm
//#define AUTODRIVE_POSITION_EPSILON_2         1600.0d

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
    /*
    leftPID(&leftInput, &leftOutput, &leftSetPoint, LEFT_Kp, LEFT_Ki, LEFT_Kd, P_ON_E, DIRECT),
    rightPID(&rightInput, &rightOutput, &rightSetPoint, RIGHT_Kp, RIGHT_Ki, RIGHT_Kd, P_ON_E, DIRECT)
    */
    linearPID(&linearInput, &linearOutput, &linearSetPoint, LINEAR_Kp, LINEAR_Ki, LINEAR_Kd, P_ON_E, DIRECT),
    rotationalPID(&rotationalInput, &rotationalOutput, &rotationalSetPoint, ROTATIONAL_Kp, ROTATIONAL_Ki, ROTATIONAL_Kd, P_ON_E, DIRECT)
{
  /*
  leftPID.SetSampleTime(sampleTime);
  leftPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
  rightPID.SetSampleTime(sampleTime);
  rightPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
  */
  linearPID.SetSampleTime(sampleTime);
  linearPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
  rotationalPID.SetSampleTime(sampleTime);
  rotationalPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
}

void FollowPath::start()
{
  /*
  leftPID.SetMode(MANUAL);
  rightPID.SetMode(MANUAL);
  */
  linearPID.SetMode(MANUAL);
  rotationalPID.SetMode(MANUAL);

  //capture our starting position
  firstPosition.set(lighthouse.getPosition());

  //setup the current segment
  currentSegmentStart = &firstPosition;
  currentSegmentEndIndex = 0;
  currentSegment.set(pathPoints[currentSegmentEndIndex].getX() - currentSegmentStart->getX(),
      pathPoints[currentSegmentEndIndex].getY() - currentSegmentStart->getY());
  currentDistanceAlongSegment = 0.0d;
  
  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  /*
  leftSetPoint = 0.0d;
  leftInput = 0.0d;
  leftOutput = 0.0d;
  rightSetPoint = 0.0d;
  rightInput = 0.0d;
  rightOutput = 0.0d;

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  */

  linearSetPoint = 0.0d;
  linearInput = 0.0d;
  linearOutput = 0.0d;
  rotationalSetPoint = 0.0d;
  rotationalInput = 0.0d;
  rotationalOutput = 0.0d;

  linearPID.SetMode(AUTOMATIC);
  rotationalPID.SetMode(AUTOMATIC);
}

bool FollowPath::loop()
{
  if (currentSegmentEndIndex+1 == pathPointCount) {
    //we're on the last path segment, so check to see if we've reached the last endpoint on that segment
    KVector2* currentPosition = lighthouse.getPosition();
    KVector2 deltaCenterToTarget(pathPoints[currentSegmentEndIndex].getX() - currentPosition->getX(),
        pathPoints[currentSegmentEndIndex].getY() - currentPosition->getY());
    if (deltaCenterToTarget.getD2() < AUTODRIVE_POSITION_EPSILON_2) {
      /*
      leftPID.SetMode(MANUAL);
      rightPID.SetMode(MANUAL);
      */
      linearPID.SetMode(MANUAL);
      rotationalPID.SetMode(MANUAL);
      return true;
    }
  }

  updateInputs();

  /*
  leftPID.Compute();
  rightPID.Compute();

  motors.setMotors(padInner(-leftOutput, MOTOR_MIN_POWER),
                   padInner(-rightOutput, MOTOR_MIN_POWER));
  */

  linearPID.Compute();
  rotationalPID.Compute();

  motors.setMotors(padInner(-(linearOutput+rotationalOutput), MOTOR_MIN_POWER),
                   padInner(-(linearOutput-rotationalOutput), MOTOR_MIN_POWER));

  return false;
}

void FollowPath::updateInputs()
{
  KVector2 nextPosition;
  calculateNextPosition(&nextPosition);

  //now make the reference to the next position relative to the local coordinate system of the robot
  KVector2* lighthouseOrientation = lighthouse.getOrientation();
  nextPosition.rotate(-lighthouseOrientation->getOrientation());

  LighthouseSensor* leftSensor = lighthouse.getLeftSensor();
  LighthouseSensor* rightSensor = lighthouse.getRightSensor();
  double angleToPosition = nextPosition.getOrientation();

  double linearVelocity = LINEAR_MAX_VELOCITY * (1.0d - constrain((abs(angleToPosition) - LINEAR_RAMPUP_END) / (LINEAR_RAMPUP_START - LINEAR_RAMPUP_END), 0.0d, 1.0d));
//  double rotationalVelocity = ROTATIONAL_MAX_VELOCITY * constrain(angleToPosition / ROTATIONAL_RAMPDOWN_START, -1.0d, 1.0d);

  /*
  double leftTargetSetPoint = linearVelocity + rotationalVelocity;
  leftSetPoint += constrain(leftTargetSetPoint - leftSetPoint, -MAX_SETPOINT_CHANGE, MAX_SETPOINT_CHANGE);
  double rightTargetSetPoint = linearVelocity - rotationalVelocity;
  rightSetPoint += constrain(rightTargetSetPoint - rightSetPoint, -MAX_SETPOINT_CHANGE, MAX_SETPOINT_CHANGE);

  leftInput = leftSensor->getVelocity();
  rightInput = rightSensor->getVelocity();
  */

  linearSetPoint = constrain(linearVelocity - linearSetPoint, -MAX_SETPOINT_CHANGE, MAX_SETPOINT_CHANGE);

  linearInput = lighthouse.getLinearVelocity();
  rotationalInput = constrain(-angleToPosition * linearInput, -ROTATIONAL_MAX_VELOCITY, ROTATIONAL_MAX_VELOCITY);
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

