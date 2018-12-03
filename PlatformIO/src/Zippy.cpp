
#include <SPI.h>
#include "Zippy.h"
#include "ZippyConfig.h"
#include "PathData.h"
#include "commands/QuadraticBezier1.h"
#include "commands/CubicBezier1.h"

#define ANGULAR_Kp                             8600.0d
#define ANGULAR_Ki                                0.0d
#define ANGULAR_Kd                              160.0d
#define ANGULAR_MAX_OUTPUT                    18000.0d

#define LINEAR_Kp                                34.0d
#define LINEAR_Ki                                 0.0d
#define LINEAR_Kd                                 6.6d
#define LINEAR_MAX_OUTPUT                     42000.0d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_EPSILON                           20.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (5 degrees)
// #define ANGULAR_EPSILON                           0.087266462599716d
//3 degrees
#define ANGULAR_EPSILON                           0.05235987755983d

//TUNING - PCM OUTPUT
//the threshold below which the wheels are set to stop entirely
#define LINEAR_MIN_THRESHOLD                     20.00d
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#define LINEAR_MIN_POWER                       4200.00d
#define ANGULAR_POWER_DELTA                    2000.00d

#define LINEAR_MAX_VELOCITY_CHANGE_FACTOR         0.10d
//the minimum amount of absolute change in linear velocity input to allow when the percentage of the previous value is below this threshold
//the application of both of these values caps acceleration changes to prevent PID overshoot
#define LINEAR_MIN_VELOCITY_CHANGE               15.00d

double clip(double a, double b)
{
  return abs(a) < b ? 0.0d : b;
}

double saturate(double a, double b)
{
  if (a == 0.0d)
    return 0.0d;

  return (a < 0.0d ? -b : b) + a ;
}

double snap(double motorPower);
double pad(double motorPower);
double lerp(double motorPower);
double relativeDistance(const KVector2* v);
double curveLength(const KVector2* v);
double curveLength(double relativeDistance, double relativeOrientation);
double constrainAcceleration(double previousValue, double newValue, double changeFactor, double minimumAcceleration);

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

void Zippy::move(double x, double y, double orientation)
{
  currentTargetPosition.vector.set(x, y);
  currentTargetPosition.orientation = orientation;
  positionUpdated = true;
  orientationUpdated = true;
  prioritizeOrientation = false;
}

void Zippy::turn(double orientation)
{
  currentTargetPosition.orientation = orientation;
  orientationUpdated = true;
  prioritizeOrientation = true;
}

void Zippy::turnAndMove(double x, double y, double orientation)
{
  currentTargetPosition.vector.set(x, y);
  currentTargetPosition.orientation = orientation;
  positionUpdated = true;
  orientationUpdated = true;
  prioritizeOrientation = true;
}

//start all of our peripherals
void Zippy::start()
{
#ifdef PLATFORM_TINYSCREEN
  motors.start();
#endif

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

  // setMotors(6600.0d, -6600.0d);
}

bool Zippy::loop(const KPosition* currentPosition,
                 const KPosition* currentPositionDelta)
{
  // /*
  angularInput = currentPositionDelta->orientation;

  KVector2 relativeVelocity(&currentPositionDelta->vector);
  double previousOrientation = subtractAngles(currentPosition->orientation, currentPositionDelta->orientation);
  relativeVelocity.rotate(-previousOrientation);
  linearInput = curveLength(&relativeVelocity);

  KVector2 relativeTargetPosition(currentTargetPosition.vector.getX() - currentPosition->vector.getX(),
      currentTargetPosition.vector.getY() - currentPosition->vector.getY());
  relativeTargetPosition.rotate(-currentPosition->orientation);
  double relativeDirectionOfMotion = subtractAngles(currentTargetPosition.orientation, currentPosition->orientation);

  double relativeOrientation;
  double relativeDistance;
  if (relativeTargetPosition.getD() < LINEAR_EPSILON) {
    if (!positionUpdated) {
      relativeOrientation = inReverse
        ? addAngles(relativeDirectionOfMotion, M_PI)
        : relativeDirectionOfMotion;
      if (!orientationUpdated &&
        abs(relativeOrientation) < ANGULAR_EPSILON &&
        currentPositionDelta->vector.getD() < LINEAR_EPSILON &&
        abs(currentPositionDelta->orientation) < ANGULAR_EPSILON)
      {
        //we are pointing toward the target orientation and moving below the desired velocity thresholds; stop moving
        angularInput = 0.0d;
        angularSetPoint = 0.0d;
        angularPID.Compute();
        linearInput = 0.0d;
        linearSetPoint = 0.0d;
        linearPID.Compute();
        setMotors(0.0d, 0.0d);
        return true;
      }
      else {
        relativeDistance = relativeTargetPosition.getY();
      }
    }
    else {
      //the target is currently moving, but we are within the linear epsilon of the moving target
      if (abs(subtractAngles(relativeDirectionOfMotion, relativeTargetPosition.getOrientation())) > M_PI_2) {
        //but the target is currently in the opposite direction of the direction that we should be moving, indicating
        //that we passed the target, so slow down
        // relativeTargetPosition.setY(-relativeTargetPosition.getY() / 2.0d);
        // relativeDistance = relativeTargetPosition.getY();
        // relativeOrientation = inReverse
          // ? addAngles(relativeTargetPosition.getOrientation(), M_PI)
          // : relativeTargetPosition.getOrientation();
        relativeOrientation = inReverse
          ? addAngles(relativeDirectionOfMotion, M_PI)
          : relativeDirectionOfMotion;
        relativeDistance = -relativeTargetPosition.getY() / 2.0d;
      }
      else {
        relativeOrientation = inReverse
          ? addAngles(relativeTargetPosition.getOrientation(), M_PI)
          : relativeTargetPosition.getOrientation();
        relativeDistance = curveLength(&relativeTargetPosition);
      }
    }
  }
  else {
    //we are not within the linear epsilon of the target position
    if (abs(subtractAngles(relativeDirectionOfMotion, relativeTargetPosition.getOrientation())) > M_PI_2) {
      //but the target is currently in the opposite direction of the direction that should be moving, indicating
      //that the target is far behind us, so face the opposite direction from the target and back up toward it
      relativeOrientation = inReverse
        ? relativeTargetPosition.getOrientation()
        : addAngles(relativeTargetPosition.getOrientation(), M_PI);
    }
    else {
      relativeOrientation = inReverse
        ? addAngles(relativeTargetPosition.getOrientation(), M_PI)
        : relativeTargetPosition.getOrientation();
    }
    relativeDistance = curveLength(&relativeTargetPosition);
  }
  positionUpdated = false;
  orientationUpdated = false;

  // angularSetPoint = constrain(relativeOrientation, -M_PI_2, M_PI_2);
  // angularSetPoint += constrain(relativeOrientation - angularSetPoint, -0.10d, 0.10d);
  angularSetPoint = constrainAcceleration(angularSetPoint, relativeOrientation, 0.1d, 0.1d);
  angularPID.Compute();

  linearSetPoint = constrainAcceleration(linearSetPoint, relativeDistance, LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE);
  // linearSetPoint += constrain(relativeDistance - linearSetPoint, -LINEAR_MIN_VELOCITY_CHANGE, LINEAR_MIN_VELOCITY_CHANGE);
  linearPID.Compute();

  driveMotors();
  // */

  return false;
}

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
  double absRelativeOrientation = abs(relativeOrientation);
  if (absRelativeOrientation == 0.0d)
    return relativeDistance;

  if (absRelativeOrientation > M_PI_2)
    absRelativeOrientation -= M_PI_2;
  return (relativeDistance * absRelativeOrientation) / sin(absRelativeOrientation);
}

/*
double centerTurnRadius(double distanceDelta, double orientationDelta)
{
  return distanceDelta / (2.0d * sin(-orientationDelta));
}
*/

void Zippy::driveMotors()
{
  double left, right;
  if (abs(linearOutput) < LINEAR_MIN_THRESHOLD) {
    if (abs(angularOutput) < LINEAR_MIN_THRESHOLD) {
      left = 0.0d;
      right = 0.0d;
    }
    else {
      left = saturate(angularOutput, LINEAR_MIN_THRESHOLD + ANGULAR_POWER_DELTA);
      right = saturate(-angularOutput, LINEAR_MIN_THRESHOLD + ANGULAR_POWER_DELTA);
    }
  }
  else {
    double saturationPoint = LINEAR_MIN_POWER;
    double absLinearOutput = abs(linearOutput);
    if (absLinearOutput < ANGULAR_POWER_DELTA) {
      saturationPoint += pow((ANGULAR_POWER_DELTA - absLinearOutput) / ANGULAR_POWER_DELTA, 4.0d) * ANGULAR_POWER_DELTA *
          min(abs(angularOutput) / absLinearOutput, 1.0d);
    }
    double base = saturate(linearOutput, saturationPoint);
    left = base + angularOutput;
    right = base - angularOutput;
  }
  setMotors(-left, -right);
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

double constrainAcceleration(double previousValue, double newValue, double changeFactor, double minimumAcceleration)
{
  double setPointDelta = newValue - previousValue;
  // /*
  double absPreviousValue = abs(previousValue);
  // if (newValue * previousValue >= 0.0d && abs(newValue) > absPreviousValue) {
  if (setPointDelta * previousValue >= 0.0d) {
    //we're accelerating either forward or backward, so limit the amount of acceleration to prevent PID overshoot
    double maxSetPointChange = max(absPreviousValue * changeFactor, minimumAcceleration);
    setPointDelta = constrain(setPointDelta, -maxSetPointChange, maxSetPointChange);
  }
  // */
  return previousValue + setPointDelta;
}
