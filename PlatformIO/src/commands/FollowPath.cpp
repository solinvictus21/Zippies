
#include "FollowPath.h"

//power to each wheel is tuned as a combination of targets for rotational power and linear power; the linear power target defines the motion forward
//or backward we wish the robot to drive and will be the same for both wheels; the rotational power target defines the offest from linear power that
//we wish to apply to force the robot to turn and will be equal magnitude but opposite direction for each wheel

//TUNING - PATH PLANNING
#define LOOP_INTERVAL_MS                         40

//TUNING - PID CONFIGURATION
/*
//at V = 500
#define LINEAR_Kp                                13.00d
#define LINEAR_Ki                                 0.00d
#define LINEAR_Kd                                 0.20d
#define ROTATIONAL_Kp                             7.40d
#define ROTATIONAL_Ki                             0.00d
#define ROTATIONAL_Kd                             0.40d
*/

// /*
//at V = 700
#define LINEAR_Kp                                11.00d
#define LINEAR_Ki                                 0.00d
#define LINEAR_Kd                                 0.50d
#define ROTATIONAL_Kp                            13.20d
#define ROTATIONAL_Ki                             0.00d
#define ROTATIONAL_Kd                             0.80d
// */

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
#define LINEAR_MIN_INPUT_CHANGE                  40.00d
#define LINEAR_MAX_INPUT_CHANGE_FACTOR            0.20d
// #define LINEAR_LOOK_AHEAD_FACTOR                  0.15d
// #define LINEAR_LOOK_AHEAD_TIME                  100

//40 degrees
// #define ROTATIONAL_RAMPDOWN_START                 0.698131700797732d
//45
// #define ROTATIONAL_RAMPDOWN_START                 0.785398163397448d
//50
// #define ROTATIONAL_RAMPDOWN_START                 0.872664625997165d
//60
#define ROTATIONAL_RAMPDOWN_START               1.047197551196598d
//80
// #define ROTATIONAL_RAMPDOWN_START                 1.396263401595464d
// 90
// #define ROTATIONAL_RAMPDOWN_START               M_PI_2
#define ROTATIONAL_MIN_VELOCITY                 350.0d

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
                       KVector2* pathPoints,
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
}

void FollowPath::start(unsigned long currentTime)
{
  // SerialUSB.println("Starting to drive.");
  zippy->recalculate();

  linearPID.SetMode(MANUAL);
  rotationalPID.SetMode(MANUAL);

  //setup the current segment
  currentControlPoint = 0;
  currentSegmentStart.set(&pathPoints[currentControlPoint]);

  //for the first bezier curve, the relative curve is from the first vertex to the midpoint between the first and second
  //vertices, and the control point is midway between the first and midpoint
  double xl = pathPoints[currentControlPoint+1].getX() - currentSegmentStart.getX();
  double yl = pathPoints[currentControlPoint+1].getY() - currentSegmentStart.getY();
  currentSegment.set(xl / 4.0d, yl / 4.0d, xl / 2.0d, yl / 2.0d);

  currentDistanceAlongSegment = 0.0d;
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
  if (currentControlPoint+1 == pathPointCount &&
      currentDistanceAlongSegment >= currentSegment.getLength())
  {
    //our time has run out
    zippy->setMotors(0.0d, 0.0d);
    // SerialUSB.println("Ended Path Following");
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
  currentDistanceAlongSegment += (LINEAR_AVG_VELOCITY * ((double)(currentTime - lastUpdateTime))) / 1000.0d;

  //determine our next target position
  KVector2 nextPosition;
  getCurrentTargetPosition(&nextPosition);

  //subtract the current position and orientation of the robot to obtain a vector relative from the robot to the target position
  nextPosition.subtractVector(zippy->getPosition());
  nextPosition.rotate(-zippy->getOrientation()->getOrientation());

  //TODO: slow down as we approach the last point on the path
  // double targetLinearVelocity = LINEAR_MAX_VELOCITY * (1.0d - constrain((abs(angleToPosition) - LINEAR_RAMPUP_END) / (LINEAR_RAMPUP_START - LINEAR_RAMPUP_END), 0.0d, 1.0d));
  double angleToPosition = nextPosition.getOrientation();
  double targetLinearVelocity = min((nextPosition.getD() * LINEAR_AVG_VELOCITY) / ((double)LOOP_INTERVAL_MS), LINEAR_MAX_VELOCITY) *
  // double targetLinearVelocity = min(nextPosition.getD() / lookAheadFactor, LINEAR_MAX_VELOCITY) *
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
      // max(ROTATIONAL_MIN_VELOCITY, abs(linearInput)-200.0d);
}

void FollowPath::getCurrentTargetPosition(KVector2* nextPosition)
{
  while (currentDistanceAlongSegment > currentSegment.getLength()) {
    //the look-ahead distance is beyond the end of the current path segment
    if (currentControlPoint+1 == pathPointCount) {
      //but this is the last segment, so stop at the end
      currentDistanceAlongSegment = currentSegment.getLength();
      break;
    }

    //there are more path segments, so move to the next path segment
    //first determine the distance left over after completing the current segment
    currentDistanceAlongSegment -= currentSegment.getLength();

    //move to the next path segment
    currentControlPoint++;
    double cx = pathPoints[currentControlPoint].getX();
    double cy = pathPoints[currentControlPoint].getY();
    //the starting point is halfway between the previous control point and the new control point
    double sx = (cx + pathPoints[currentControlPoint-1].getX()) / 2.0d;
    double sy = (cy + pathPoints[currentControlPoint-1].getY()) / 2.0d;
    currentSegmentStart.set(sx, sy);
    double ex = cx;
    double ey = cy;
    if (currentControlPoint+1 == pathPointCount) {
      //at the end of the path, the endpoint is the last control point
      ex = cx;
      ey = cy;
      //and the control point is halfway to the endpoint
      cx = (cx + sx) / 2.0d;
      cy = (cy + sy) / 2.0d;
    }
    else {
      //prior to the end of the path, the endpoint is halway between the current control point and the next control point
      ex = (ex + pathPoints[currentControlPoint+1].getX()) / 2.0d;
      ey = (ey + pathPoints[currentControlPoint+1].getY()) / 2.0d;
    }

    //our current segment is the relative path (starting at 0.0) we'll be following now
    currentSegment.set(cx - sx, cy - sy, ex - sx, ey - sy);
  }

  //start by determining the current distance we've traveled along this segment, normalized from 0.0 to 1.0
  double deltaAlongPath = currentDistanceAlongSegment / currentSegment.getLength();

  //now LERP out the x/y position along the current path segment
  currentSegment.lerpPoint(currentDistanceAlongSegment, nextPosition);
  nextPosition->addVector(&currentSegmentStart);
  // nextPosition->printDebug();
  // nextPosition->set(currentSegmentStart->getX() + (currentSegment.getX() * deltaAlongPath),
      // currentSegmentStart->getY() + (currentSegment.getY() * deltaAlongPath));
}
