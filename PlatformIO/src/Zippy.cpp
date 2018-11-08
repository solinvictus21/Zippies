
#include <SPI.h>
#include "Zippy.h"
#include "ZippyConfig.h"
#include "PathData.h"
#include "commands/QuadraticBezier1.h"
#include "commands/CubicBezier1.h"

#define WHEEL_OFFSET_X   16.7d
// #define WHEEL_OFFSET_X   30.0d
#define WHEEL_OFFSET_Y    5.9d
// const double wheelOffset = sqrt(pow(WHEEL_OFFSET_X, 2.0d) + pow(WHEEL_OFFSET_Y, 2.0d));
// const double axleOrientationOffset = atan2(-WHEEL_OFFSET_Y, WHEEL_OFFSET_X);

//TUNING - COMMAND EXECUTION COMPLETION
//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_EPSILON                           30.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (5 degrees)
#define ANGULAR_EPSILON                           0.087266462599716d
//3 degrees
// #define ANGULAR_EPSILON                           0.05235987755983d

//TUNING - PCM OUTPUT
//the threshold below which the wheels are set to stop entirely
#define LINEAR_MIN_THRESHOLD                     50.00d
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#define LINEAR_MIN_POWER                       3200.00d
// #define LINEAR_MIN_POWER                       2000.00d

#define LINEAR_MAX_VELOCITY_CHANGE_FACTOR         0.20d
//the minimum amount of absolute change in linear velocity input to allow when the percentage of the previous value is below this threshold
//the application of both of these values caps acceleration changes to prevent PID overshoot
#define LINEAR_MIN_VELOCITY_CHANGE               50.00d

double snap(double motorPower);
double pad(double motorPower);
double lerp(double motorPower);
double relativeDistance(const KVector2* v);
double centerTurnRadius(double distanceDelta, double orientationDelta);
double curveLength(const KVector2* v);
double curveLength(double relativeDistance, double relativeOrientation);

#ifdef INDEPENDENT_WHEEL_PIDS
Zippy::Zippy(unsigned long pui)
  : pidUpdateInterval(pui),
    leftWheel(-WHEEL_OFFSET_X, -WHEEL_OFFSET_Y, pidUpdateInterval),
    rightWheel(WHEEL_OFFSET_X, WHEEL_OFFSET_Y, pidUpdateInterval)
{
  face.displayFace();
}
#else
#define ANGULAR_Kp                             5000.0d
#define ANGULAR_Ki                                0.0d
#define ANGULAR_Kd                                0.0d
#define ANGULAR_MAX_OUTPUT                    15000.0d
#define LINEAR_Kp                                20.0d
#define LINEAR_Ki                                 0.0d
#define LINEAR_Kd                                 0.0d
#define LINEAR_MAX_OUTPUT                     45000.0d
Zippy::Zippy(unsigned long pui)
  : angularPID(&angularInput, &angularOutput, &angularSetPoint, ANGULAR_Kp, ANGULAR_Ki, ANGULAR_Kd, P_ON_E, DIRECT),
    linearPID(&linearInput, &linearOutput, &linearSetPoint, LINEAR_Kp, LINEAR_Ki, LINEAR_Kd, P_ON_E, DIRECT)
{
  angularPID.SetSampleTime(pui);
  angularPID.SetOutputLimits(-ANGULAR_MAX_OUTPUT, ANGULAR_MAX_OUTPUT);
  linearPID.SetSampleTime(pui);
  linearPID.SetOutputLimits(-LINEAR_MAX_OUTPUT, LINEAR_MAX_OUTPUT);
  face.displayFace();
}
#endif

void Zippy::move(double x, double y, double orientation)
{
  currentTargetPosition.vector.set(x, y);
  currentTargetPosition.orientation = orientation;
  positionUpdated = true;
  orientationUpdated = true;
  prioritizeOrientation = false;
  stopped = false;
}

void Zippy::turn(double orientation)
{
  currentTargetPosition.orientation = orientation;
  orientationUpdated = true;
  prioritizeOrientation = true;
  stopped = false;
}

void Zippy::turnAndMove(double x, double y, double orientation)
{
  currentTargetPosition.vector.set(x, y);
  currentTargetPosition.orientation = orientation;
  positionUpdated = true;
  orientationUpdated = true;
  prioritizeOrientation = true;
  stopped = false;
}

//start all of our peripherals
void Zippy::start()
{
#ifdef PLATFORM_TINYSCREEN
  motors.start();
#endif

#ifdef INDEPENDENT_WHEEL_PIDS
  leftWheel.start();
  rightWheel.start();
#else
  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  angularPID.SetMode(MANUAL);
  angularSetPoint = 0.0d;
  angularInput = 0.0d;
  angularOutput = 0.0d;
  angularPID.SetMode(AUTOMATIC);
  linearPID.SetMode(MANUAL);
  linearSetPoint = 0.0d;
  linearInput = 0.0d;
  linearOutput = 0.0d;
  linearPID.SetMode(AUTOMATIC);
#endif
}

bool Zippy::loop(const KPosition* currentPosition,
                 const KPosition* currentPositionDelta)
{
  KVector2 relativeVelocity(&currentPositionDelta->vector);
  double previousOrientation = subtractAngles(currentPosition->orientation, currentPositionDelta->orientation);
  relativeVelocity.rotate(-previousOrientation);
  double relativeVelocityDistance = relativeDistance(&relativeVelocity);
#ifdef INDEPENDENT_WHEEL_PIDS
  updateInputs(relativeVelocityDistance, relativeVelocity.getOrientation());
#endif

  KVector2 relativeTargetPosition(currentTargetPosition.vector.getX() - currentPosition->vector.getX(),
      currentTargetPosition.vector.getY() - currentPosition->vector.getY());
  relativeTargetPosition.rotate(-currentPosition->orientation);
  double relativeTargetDistance = relativeDistance(&relativeTargetPosition);

  // /*
  //keep downward pressure on large increases in velocity to prevent PID overshoot
  boolean sameDirection = relativeTargetDistance * linearSetPoint >= 0.0d;
  if (!sameDirection ||
      (sameDirection && relativeTargetPosition.getD() > abs(linearSetPoint)))
  {
    double velocityChange = relativeTargetDistance - linearSetPoint;
    double maxVelocityChange = max(relativeVelocity.getD() * LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE);
    relativeTargetDistance = relativeVelocityDistance + constrain(velocityChange, -maxVelocityChange, maxVelocityChange);
  }
  // */

  double relativeOrientation;
  if (!positionUpdated &&
      relativeTargetPosition.getD() < LINEAR_EPSILON)
  {
    //we're at the target position and the position has not changed, so just turn in place while inching forward
    //or backward just enough to keep the Zippy pinned to the target position
    relativeOrientation = subtractAngles(currentTargetPosition.orientation, currentPosition->orientation);
    /*
    if (inReverse)
      relativeOrientation = addAngles(relativeOrientation, M_PI);
    // */

    //we are at the target position
    if (!orientationUpdated &&
        abs(relativeOrientation) < ANGULAR_EPSILON)// &&
        // currentPositionDelta->vector.getD() < LINEAR_EPSILON &&
        // abs(currentPositionDelta->orientation) < ANGULAR_EPSILON)
    {
      //we are pointing toward the target orientation and moving below the desired velocity threshold; stop moving
#ifdef INDEPENDENT_WHEEL_PIDS
      leftWheel.stop();
      rightWheel.stop();
#else
      angularInput = 0.0d;
      angularSetPoint = 0.0d;
      angularPID.Compute();
      linearInput = 0.0d;
      linearSetPoint = 0.0d;
      linearPID.Compute();
#endif
      setMotors(0.0d, 0.0d);
      return true;
    }
  }
  else {
    relativeOrientation = relativeTargetPosition.getOrientation();
    /*
    if (inReverse) {
      if (relativeTargetPosition.getY() > 0.0d && abs(subtractAngles(currentPosition->orientation, currentTargetPosition.orientation)) > M_PI_2) {
        relativeTargetPosition.setY(-relativeTargetPosition.getY());
        relativeTargetPosition.setD(relativeTargetPosition.getD() / 2.0d);
        relativeOrientation = relativeTargetPosition.getOrientation();
      }
      relativeOrientation = addAngles(relativeOrientation, M_PI);
    }
    else if (relativeTargetPosition.getY() < 0.0d && abs(subtractAngles(currentPosition->orientation, currentTargetPosition.orientation)) <= M_PI_2) {
      //the position is currently moving, and we're pointing in the direction that the position is moving toward, but we've
      //passed the current target position; in that situation, just turn back toward the target orientation and slow down
      // face.clearScreen();
      relativeTargetPosition.setY(-relativeTargetPosition.getY());
      relativeTargetPosition.setD(relativeTargetPosition.getD() / 2.0d);
      relativeOrientation = relativeTargetPosition.getOrientation();
    }
    // */
  }
  positionUpdated = false;
  orientationUpdated = false;

#ifdef INDEPENDENT_WHEEL_PIDS
  double turnRadius = relativeTargetPosition.getY() <= 0.0d
    ? 0.0d
    : centerTurnRadius(relativeTargetDistance, relativeOrientation);
  leftWheel.move(
    turnRadius,
    relativeOrientation);
  rightWheel.move(
    turnRadius,
    relativeOrientation);

  double left = leftWheel.getOutput();
  double right = rightWheel.getOutput();
#else
  angularInput = currentPositionDelta->orientation;
  angularSetPoint = relativeOrientation;
  angularPID.Compute();
  linearInput = curveLength(&relativeVelocity);
  linearSetPoint = curveLength(relativeTargetDistance, relativeTargetPosition.getOrientation());
  // linearSetPoint = curveLength(&relativeTargetPosition);
  linearPID.Compute();

  // double left = angularOutput;
  // double right = -angularOutput;
  double left = linearOutput + angularOutput;
  double right = linearOutput - angularOutput;
#endif
  // if (abs((left + right) / 2.0d) < LINEAR_MIN_POWER) {
    left = snap(left);
    right = snap(right);
    // left = lerp(left);
    // right = lerp(right);
  // }
  setMotors(-left, -right);

  return false;
}

#ifdef INDEPENDENT_WHEEL_PIDS
void Zippy::updateInputs(double relativeVelocityDistance, double relativeOrientation)
{
  if (relativeOrientation == 0.0d) {
    leftWheel.setInput(relativeVelocityDistance);
    rightWheel.setInput(relativeVelocityDistance);
    return;
  }

  double velocityTurnRadius = centerTurnRadius(relativeVelocityDistance, relativeOrientation);
  leftWheel.setInput(velocityTurnRadius, relativeOrientation);
  rightWheel.setInput(velocityTurnRadius, relativeOrientation);
}
#endif

double relativeDistance(const KVector2* v)
{
  double d = v->getD();
  return v->getY() >= 0.0d ? d : -d;
}

double curveLength(const KVector2* v)
{
  return curveLength(relativeDistance(v), v->getOrientation());
}

double curveLength(double relativeDistance, double relativeOrientation)
{
  if (relativeOrientation == 0.0d)
    return relativeDistance;

  return (relativeDistance * relativeOrientation) / sin(relativeOrientation);
}

double centerTurnRadius(double distanceDelta, double orientationDelta)
{
  return distanceDelta / (2.0d * sin(-orientationDelta));
}

void Zippy::stop()
{
  setMotors(0.0d, 0.0d);
  stopped = true;
}

void Zippy::setMotors(int32_t motorLeft, int32_t motorRight)
{
  motors.writeCommand(COMMAND_ALL_PWM,
      motorLeft < 0 ? -motorLeft: 0,
      motorLeft > 0 ? motorLeft: 0,
      motorRight < 0 ? -motorRight: 0,
      motorRight > 0 ? motorRight: 0);
}

double lerp(double motorPower)
{
  double absMotorPower = abs(motorPower);
  if (absMotorPower > LINEAR_MIN_POWER)
    return motorPower;

  double lerp = quadraticLerp0(absMotorPower / LINEAR_MIN_POWER, LINEAR_MIN_POWER, LINEAR_MIN_POWER - (0.1d * LINEAR_MIN_POWER));
  return motorPower >= 0.0d ? lerp : -lerp;
}

double snap(double motorPower)
{
  double absMotorPower = abs(motorPower);
  if (absMotorPower < LINEAR_MIN_THRESHOLD)
    return 0.0d;

  absMotorPower -= LINEAR_MIN_THRESHOLD;
  double power = LINEAR_MIN_POWER + absMotorPower;
  return motorPower < 0.0d ? -power : power;
}

double pad(double motorPower)
{
  if (motorPower > 0.0d)
    return LINEAR_MIN_POWER + motorPower;
  else if (motorPower < 0.0d)
    return -LINEAR_MIN_POWER + motorPower;

  return 0.0d;
}
