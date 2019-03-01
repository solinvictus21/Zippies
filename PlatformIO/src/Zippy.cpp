
#include <SPI.h>
#include "Zippy.h"
#include "PathData.h"
#include "commands/QuadraticBezier1.h"
#include "commands/CubicBezier1.h"

#ifdef KINEMATIC_MODEL_INDEPENDENT
#define WHEEL_OFFSET_X   16.7d
#define WHEEL_OFFSET_Y    5.9d
#define M_PI_34 2.356194490192345d

double centerTurnRadius(double distanceDelta, double orientationDelta)
{
  return distanceDelta / (2.0d * sin(-orientationDelta));
}
#else
#define ANGULAR_Kp                            10000.0d
#define ANGULAR_Ki                                0.0d
#define ANGULAR_Kd                              130.0d
#define ANGULAR_MAX_OUTPUT                    18000.0d

#define LINEAR_Kp                                42.0d
#define LINEAR_Ki                                 0.0d
#define LINEAR_Kd                                 5.0d
#define LINEAR_MAX_OUTPUT                     42000.0d

#define ANGULAR_POWER_DELTA                    2000.00d
#endif

//TUNING - COMMAND EXECUTION COMPLETION
//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_POSITION_EPSILON                           30.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (5 degrees)
// #define ANGULAR_POSITION_EPSILON                  0.087266462599716d
//3 degrees
#define ANGULAR_POSITION_EPSILON                  0.05235987755983d

//TUNING - PCM OUTPUT
//the threshold below which the wheels are set to stop entirely
#define POWER_MIN_THRESHOLD                      10.00d
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#define LINEAR_MIN_POWER                       4200.00d

//the minimum amount of absolute change in linear velocity input to allow when the percentage of the previous value is below
//this threshold the application of both of these values caps acceleration changes to prevent PID overshoot
#define LINEAR_MIN_VELOCITY_CHANGE                4.00d
#define LINEAR_MAX_VELOCITY_CHANGE_FACTOR         0.30d
#define LINEAR_MAX_VELOCITY                      40.00d

double saturate(double a, double b);
double relativeDistance(const KVector2* v);
double curveLength(const KVector2* v);
double curveLength(double d, double o);
double constrainAcceleration(double previousValue, double newValue, double changeFactor, double minimumAcceleration);

// double clip(double a, double b);
// double snap(double motorPower);
// double pad(double motorPower);
// double lerp(double motorPower);
// double curveLength(double relativeDistance, double relativeOrientation);

Zippy::Zippy(unsigned long pui)
#ifdef KINEMATIC_MODEL_INDEPENDENT
  : leftWheel(-WHEEL_OFFSET_X, -WHEEL_OFFSET_Y, pui),
    rightWheel(WHEEL_OFFSET_X, WHEEL_OFFSET_Y, pui)
{
#else
  : angularPID(&angularInput, &angularOutput, &angularSetPoint, ANGULAR_Kp, ANGULAR_Ki, ANGULAR_Kd, P_ON_E, DIRECT),
    linearPID(&linearInput, &linearOutput, &linearSetPoint, LINEAR_Kp, LINEAR_Ki, LINEAR_Kd, P_ON_E, DIRECT)
{
  angularPID.SetSampleTime(pui);
  angularPID.SetOutputLimits(-ANGULAR_MAX_OUTPUT, ANGULAR_MAX_OUTPUT);
  linearPID.SetSampleTime(pui);
  linearPID.SetOutputLimits(-LINEAR_MAX_OUTPUT, LINEAR_MAX_OUTPUT);
#endif

#ifdef PLATFORM_TINYSCREEN
  face.displayFace();
#endif
}

#ifdef ENABLE_SDCARD_LOGGING
void Zippy::startLogging()
{
  loggingEnabled = true;
}

void Zippy::stopLogging()
{
  loggingEnabled = false;
}
#endif

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
  motors.start();

#ifdef KINEMATIC_MODEL_INDEPENDENT
  leftWheel.start();
  rightWheel.start();

  currentTargetVelocity = 0.0d;
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

  // setMotors(6600.0d, -6600.0d);
  // setMotors(0.0d, -6400.0d);
#ifdef ENABLE_SDCARD_LOGGING
  if (!SD.begin(SD_CHIP_SELECT)) {
    SerialUSB.println("Card failed, or not present");
  }
  else
    SerialUSB.println("Connected to SD card.");
#endif
}

bool Zippy::loop(const KPosition* currentPosition,
                 const KPosition* currentPositionDelta)
{
  KVector2 relativeVelocity(&currentPositionDelta->vector);
  double previousOrientation = subtractAngles(currentPosition->orientation, currentPositionDelta->orientation);
  relativeVelocity.rotate(-previousOrientation);

  double relativeDirectionOfMotion = subtractAngles(currentTargetPosition.orientation, currentPosition->orientation);
  double relativeTargetOrientation = relativeDirectionOfMotion;
  if (inReverse)
    relativeTargetOrientation = addAngles(relativeTargetOrientation, M_PI);

  KVector2 relativeTargetPosition(currentTargetPosition.vector.getX() - currentPosition->vector.getX(),
      currentTargetPosition.vector.getY() - currentPosition->vector.getY());
  relativeTargetPosition.rotate(-currentPosition->orientation);

#ifdef KINEMATIC_MODEL_INDEPENDENT
  /*
  leftWheel.setInput(&relativeVelocity, currentPositionDelta->orientation);
  rightWheel.setInput(&relativeVelocity, currentPositionDelta->orientation);
  // */
  double relativeVelocityDistance = relativeDistance(&relativeVelocity);
  double relativeVelocityOrientation = relativeVelocity.getOrientation();
  if (relativeVelocity.getY() < 0.0d)
    relativeVelocityOrientation = addAngles(relativeVelocityOrientation, M_PI);
  double velocityTurnRadius = centerTurnRadius(relativeVelocityDistance, relativeVelocityOrientation);
  leftWheel.setInput2(velocityTurnRadius, relativeVelocityOrientation);
  rightWheel.setInput2(velocityTurnRadius, relativeVelocityOrientation);
#else
  angularInput = currentPositionDelta->orientation;
  linearInput = curveLength(&relativeVelocity);
#endif

  if (relativeTargetPosition.getD() < LINEAR_POSITION_EPSILON) {
    if (!positionUpdated) {
      if (!orientationUpdated &&
        abs(relativeTargetOrientation) < ANGULAR_POSITION_EPSILON/* &&
        currentTargetVelocity < LINEAR_POSITION_EPSILON &&
        currentPositionDelta->vector.getD() < LINEAR_EPSILON &&
        abs(currentPositionDelta->orientation) < ANGULAR_POSITION_EPSILON*/)
      {
        //the position and orientation have stopped, we're in position and oriented and we're going slow enough to stop
        driveStop();
        return true;
      }

      //the position is not changing and we're near enough to it to just turn in place while inching forward or backward
      //to stay pinned to the appropriate spot
      driveTurn(relativeTargetOrientation);
      // driveStop();
      // return true;
    }
    //we're too near the target to drive normally because we could easily over-steer
    else
      driveCurved(&relativeTargetPosition, relativeTargetOrientation, relativeDirectionOfMotion);
  }
  else
    driveCurved(&relativeTargetPosition, relativeTargetOrientation, relativeDirectionOfMotion);

  positionUpdated = false;
  orientationUpdated = false;

  driveMotors();

  return false;
}

/*
void Zippy::turnNear()
{
  if (!orientationUpdated &&
    currentPositionDelta->vector.getD() < LINEAR_EPSILON &&
    abs(relativeOrientation) < ANGULAR_EPSILON &&
    abs(currentPositionDelta->orientation) < ANGULAR_EPSILON)
  {
    //we're within the required epsilon of the target position, neither the position nor the orientation are
    //changing, and we're currently moving slowly enough that we can just stop moving
    leftWheel.stop();
    rightWheel.stop();
    setMotors(0.0d, 0.0d);
    return true;
  }

  double relativeDistance = relativeDistance(&relativeTargetPosition);
  double turnRadius = centerTurnRadius(relativeDistance, relativeTargetOrientation);
  leftWheel.move(turnRadius, relativeDistance);
  rightWheel.move(turnRadius, relativeDistance);
  driveMotors();
}
*/

#ifdef KINEMATIC_MODEL_INDEPENDENT
void Zippy::driveTurn(double relativeTargetOrientation)
{
  currentTargetVelocity = 0.0d;
  relativeTargetOrientation = constrain(relativeTargetOrientation, -M_PI_4, M_PI_4);
  /*
  leftWheel.turn(relativeTargetOrientation);
  rightWheel.turn(relativeTargetOrientation);
  // */
  // leftWheel.move2(0.0d, relativeTargetOrientation);
  // rightWheel.move2(0.0d, relativeTargetOrientation);
  leftWheel.turn2(relativeTargetOrientation);
  rightWheel.turn2(relativeTargetOrientation);
}

/*
void Zippy::driveNear(KVector2* relativeTargetPosition, double relativeDirectionOfMotion)
{
  double relativeTargetDirection = relativeTargetPosition->getOrientation();
  //we are within the linear epsilon of the target, and the target is currently in motion
  if (abs(subtractAngles(relativeDirectionOfMotion, relativeTargetDirection)) > M_PI_2) {
    //...but the target is currently in the opposite direction of the direction we want to move, indicating
    //that we've passed the target; slow down and turn back toward the target
    relativeTargetPosition->set(0.0d, relativeTargetPosition->getD() / 2.0d);
    relativeTargetPosition->rotate(relativeDirectionOfMotion);
    relativeTargetDirection = relativeDirectionOfMotion;
  }

  if (inReverse)
    relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);
  if (relativeTargetDirection >= M_PI_2) {
    driveTurn(relativeTargetDirection);
    return;
  }

  driveNormal(relativeTargetPosition);
}
*/

/*
//the "normal" use case; we are not within the linear epsilon of the target position; just drive toward it
void Zippy::driveFar(KVector2* relativeTargetPosition, double relativeDirectionOfMotion)
{
  double relativeTargetDirection = relativeTargetPosition->getOrientation();
  //we are within the linear epsilon of the target, and the target is currently in motion
  if (abs(subtractAngles(relativeDirectionOfMotion, relativeTargetDirection)) > M_PI_2) {
  // if ((abs(relativeDirectionOfMotion) < M_PI_2) != (abs(relativeTargetDirection) < M_PI_2)) {
    //...but the target is currently in the opposite direction of the direction we want to move; reverse
    //course toward the target
    relativeTargetPosition->set(0.0d, relativeDistance(relativeTargetPosition));
    relativeTargetDirection = (relativeTargetPosition->getY() >= 0.0d) ? 0.0d : M_PI;
  }

  if (inReverse)
    relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);
  if (abs(relativeTargetDirection) >= M_PI_2) {
    driveTurn(relativeTargetDirection);
    return;
  }

  driveNormal(relativeTargetPosition);
}
*/

void Zippy::driveCurved(KVector2* relativeTargetPosition, double relativeTargetOrientation, double relativeDirectionOfMotion)
{
  /*
  // bool motionBehind = abs(relativeDirectionOfMotion) > M_PI_2;
  bool targetAhead = abs(relativeTargetDirection) <= M_PI_2;
  bool targetTowardMotion = abs(subtractAngles(relativeDirectionOfMotion, relativeTargetDirection)) <= M_PI_2;
  if (abs(relativeTargetOrientation) > M_PI_2) {
    //we are incorrectly oriented; turn toward the appropriate direction
    if (!targetTowardMotion)
      relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);
    driveTurn(relativeTargetDirection);
    return;
  }
  // */

  // /*
  double idealVelocity = relativeDistance(relativeTargetPosition);
  double relativeTargetDirection = relativeTargetPosition->getOrientation();
  // */
  /*
  if (abs(relativeTargetOrientation) > M_PI_2) {
    //we're facing the wrong way; turn around
    // if (abs(relativeTargetDirection) <= M_PI_4)
    if (abs(subtractAngles(relativeTargetDirection, relativeTargetOrientation)) > M_PI_2)
      relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);
    driveTurn(relativeTargetDirection);
    return;
  }
  // */

  /*
  double targetToMotion = subtractAngles(relativeDirectionOfMotion, relativeTargetDirection);
  double absTargetToMotion = abs(targetToMotion);
  if (absTargetToMotion > M_PI_4) {
    // double turnRatio = absTargetToMotion / M_PI;
    // double turnRatio = (absTargetToMotion - M_PI_2) / M_PI_2;
    double turnRatio = (absTargetToMotion - M_PI_4) / (3.0d * M_PI_4);
    idealVelocity *= (1.0d - turnRatio);
    relativeTargetDirection = addAngles(relativeTargetDirection, targetToMotion * turnRatio);
  }
  // */
  /*
  else if (abs(subtractAngles(relativeTargetPosition->getOrientation(), relativeDirectionOfMotion)) > M_PI_2) {
    //the target position is not in the direction of motion; slow down and turn toward the direction of motion
    currentTargetVelocity /= 2.0d;
    relativeTargetDirection = relativeDirectionOfMotion / 2.0d;
  }
  // */
  // /*
  currentTargetVelocity = constrain(constrainAcceleration(currentTargetVelocity, idealVelocity,
      LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE),
      -LINEAR_MAX_VELOCITY, LINEAR_MAX_VELOCITY);

  /*
  if (currentTargetVelocity < 0.0d)
    relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);
  // */

  /*
  //we are oriented correctly; make sure we didn't pass the target
  if (targetAhead && targetTowardMotion) {
    currentTargetVelocity = constrain(constrainAcceleration(currentTargetVelocity, relativeDistance(relativeTargetPosition),
        LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE),
        -LINEAR_MAX_VELOCITY, LINEAR_MAX_VELOCITY);
  }
  else {
    currentTargetVelocity *= 0.8d;
    if (!targetAhead)
      relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);
  }

  leftWheel.move(currentTargetVelocity, relativeTargetDirection);
  rightWheel.move(currentTargetVelocity, relativeTargetDirection);
  // */

  double turnRadius = relativeTargetPosition->getY() <= 0.0d
    ? 0.0d
    : centerTurnRadius(currentTargetVelocity, relativeTargetDirection);
  leftWheel.move2(
    turnRadius,
    relativeTargetDirection);
  rightWheel.move2(
    turnRadius,
    relativeTargetDirection);
}

/*
void Zippy::driveNormal(KVector2* relativeTargetPosition)
{
  double relativeTargetDistance = relativeDistance(relativeTargetPosition);
  currentTargetVelocity = constrain(constrainAcceleration(currentTargetVelocity, relativeTargetDistance,
      LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE),
      -LINEAR_MAX_VELOCITY, LINEAR_MAX_VELOCITY);
  if (relativeTargetDistance != currentTargetVelocity)
    relativeTargetPosition->setD(abs(currentTargetVelocity));

  leftWheel.move(relativeTargetPosition);
  rightWheel.move(relativeTargetPosition);
}
*/

void Zippy::driveStop()
{
  currentTargetVelocity = 0.0d;
  leftWheel.stop();
  rightWheel.stop();
  // setMotors(0.0d, 0.0d);
  motors.writeCommand(COMMAND_ALL_PWM, 10000, 10000, 10000, 10000);
}

#else

void Zippy::driveTurn(double relativeTargetOrientation)
{
  angularSetPoint = constrainAcceleration(angularSetPoint, relativeTargetOrientation, 0.1d, 0.1d);
  angularPID.Compute();

  // linearSetPoint = constrainAcceleration(linearSetPoint, relativeTargetPosition->getY(),
      // LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE);
  linearSetPoint = linearInput;
  linearPID.Compute();
}

void Zippy::driveNear(KVector2* relativeTargetPosition, double relativeDirectionOfMotion)
{
  //we are within the linear epsilon of the target, but the target is currently in motion
  double relativeTargetDistance;
  double relativeTargetOrientation;
  if (abs(subtractAngles(relativeDirectionOfMotion, relativeTargetPosition->getOrientation())) > M_PI_2) {
    //the target is currently in the opposite direction of the direction that we should be moving, indicating
    //that we passed the target, so keep moving in the same direction, but slow down
    relativeTargetDistance = -relativeTargetPosition->getY() / 2.0d;
    relativeTargetOrientation = relativeDirectionOfMotion;
  }
  else {
    relativeTargetDistance = curveLength(relativeTargetPosition);
    relativeTargetOrientation = relativeTargetPosition->getOrientation();
  }

  if (inReverse)
    relativeTargetOrientation = addAngles(relativeTargetOrientation, M_PI);

  angularSetPoint = constrainAcceleration(angularSetPoint, relativeTargetOrientation, 0.1d, 0.1d);
  angularPID.Compute();

  linearSetPoint = constrainAcceleration(linearSetPoint, relativeTargetOrientation,
    LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE);
  linearPID.Compute();
}

void Zippy::driveFar(KVector2* relativeTargetPosition, double relativeDirectionOfMotion)
{
  double relativeTargetDirection = relativeTargetPosition->getOrientation();
  //we are within the linear epsilon of the target, and the target is currently in motion
  if (abs(subtractAngles(relativeDirectionOfMotion, relativeTargetDirection)) > M_PI_2) {
    //...but the target is currently in the opposite direction of the direction we want to move, indicating
    //that we've passed the target; slow down and turn back toward the target
    if (!inReverse)
      relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);
  }
  else if (inReverse)
      relativeTargetDirection = addAngles(relativeTargetDirection, M_PI);

  if (relativeTargetDirection >= M_PI_2) {
    driveTurn(relativeTargetDirection);
    return;
  }

  driveNormal(relativeTargetPosition);
}

//the "normal" use case; we are not within the linear epsilon of the target position; just drive toward it
void Zippy::driveNormal(KVector2* relativeTargetPosition)
{
  //we are not within the linear epsilon of the target position
  double relativeTargetDistance = curveLength(relativeTargetPosition);
  double relativeTargetOrientation = relativeTargetPosition->getOrientation();
  /*
  if (abs(subtractAngles(relativeDirectionOfMotion, relativeTargetOrientation)) > M_PI_2) {
    //but the target is currently in the opposite direction of the direction that should be moving, indicating
    //that the target is far behind us, so face the opposite direction from the target and back up toward it
    if (!inReverse)
      relativeTargetOrientation = addAngles(relativeTargetOrientation, M_PI);
  }
  else if (inReverse)
    relativeTargetOrientation = addAngles(relativeTargetOrientation, M_PI);
  */

  angularSetPoint = constrainAcceleration(angularSetPoint, relativeTargetOrientation, 0.1d, 0.1d);
  angularPID.Compute();

  linearSetPoint = constrainAcceleration(linearSetPoint, relativeTargetDistance,
      LINEAR_MAX_VELOCITY_CHANGE_FACTOR, LINEAR_MIN_VELOCITY_CHANGE);
  linearPID.Compute();
}

void Zippy::driveStop()
{
  angularSetPoint = angularInput;
  angularPID.Compute();
  linearSetPoint = linearInput;
  linearPID.Compute();
  // setMotors(0.0d, 0.0d);
  motors.writeCommand(COMMAND_ALL_PWM, 10000, 10000, 10000, 10000);
}
#endif

void Zippy::driveMotors()
{
#ifdef KINEMATIC_MODEL_INDEPENDENT
  double left = leftWheel.getOutput();
  double right = rightWheel.getOutput();
  /*
  if (abs(left) < POWER_MIN_THRESHOLD && abs(right) < POWER_MIN_THRESHOLD) {
    setMotors(0.0d, 0.0d);
    return;
  }
  */

  left = saturate(left, LINEAR_MIN_POWER);
  right = saturate(right, LINEAR_MIN_POWER);
#else
  double absLinearOutput = abs(linearOutput);
  double absAngularOutput = abs(angularOutput);
  if (absLinearOutput < POWER_MIN_THRESHOLD && absAngularOutput < POWER_MIN_THRESHOLD) {
    // setMotors(0.0d, 0.0d);
    motors.writeCommand(COMMAND_ALL_PWM, 10000, 10000, 10000, 10000);
    return;
  }

  /*
  double saturationPoint = LINEAR_MIN_POWER;

  if (absLinearOutput > absAngularOutput) {
    //both wheels are going to turn in the same direction
    // saturationPoint += ANGULAR_POWER_DELTA / (absLinearOutput / (absLinearOutput + absAngularOutput));
    saturationPoint += ANGULAR_POWER_DELTA * (absAngularOutput / absLinearOutput);
  }
  else {
    //wheels are moving in opposite directions
    saturationPoint += ANGULAR_POWER_DELTA;
  }
  double left = saturate(linearOutput + angularOutput, saturationPoint);
  double right = saturate(linearOutput - angularOutput, saturationPoint);
  */
  linearOutput = saturate(linearOutput, LINEAR_MIN_POWER);
  double left = linearOutput + angularOutput;
  double right = linearOutput - angularOutput;
#endif
  // */

  /*
  //turn harder at slower speeds, because power is lost due to fact that the wheels are not tangential to the turn radius
  if (absLinearOutput < ANGULAR_POWER_DELTA) {
    //the saturated linear output will be less than LINEAR_MIN_POWER + ANGULAR_POWER_DELTA
    saturationPoint += pow((ANGULAR_POWER_DELTA - absLinearOutput) / ANGULAR_POWER_DELTA, 4.0d) * ANGULAR_POWER_DELTA *
        min(absAngularOutput / absLinearOutput, 1.0d);
  }
  double base = saturate(linearOutput, saturationPoint);
  double left = base + angularOutput;
  double right = base - angularOutput;
  // */

  setMotors(left, right);
}

void Zippy::setMotors(int32_t motorLeft, int32_t motorRight)
{
  motors.writeCommand(COMMAND_ALL_PWM,
      motorLeft > 0 ? motorLeft : 0,
      motorLeft < 0 ? -motorLeft : 0,
      motorRight > 0 ? motorRight : 0,
      motorRight < 0 ? -motorRight : 0);
}

double relativeDistance(const KVector2* v)
{
  double d = v->getD();
  return v->getY() >= 0.0d ? d : -d;
}

double curveLength(double relativeDistance, double relativeOrientation)
{
  double absRelativeOrientation = abs(relativeOrientation);
  if (absRelativeOrientation > M_PI_2)
    absRelativeOrientation = abs(absRelativeOrientation - M_PI);
  if (absRelativeOrientation == 0.0d)
    return relativeDistance;

  return (relativeDistance * absRelativeOrientation) / sin(absRelativeOrientation);
}

double curveLength(const KVector2* v)
{
  return curveLength(relativeDistance(v), v->getOrientation());
}

double constrainAcceleration(double previousValue, double newValue, double changeFactor, double minimumAcceleration)
{
  double setPointDelta = newValue - previousValue;
  if (setPointDelta * previousValue >= 0.0d) {
    //we're accelerating either forward or backward, so limit the amount of acceleration to prevent PID overshoot
    double maxSetPointChange = max(abs(previousValue) * changeFactor, minimumAcceleration);
    setPointDelta = constrain(setPointDelta, -maxSetPointChange, maxSetPointChange);
  }
  return previousValue + setPointDelta;
}

double saturate(double a, double b)
{
  if (a == 0.0d)
    return 0.0d;

  return (a < 0.0d ? -b : b) + a ;
}

/*
double clip(double a, double b)
{
  return abs(a) < b ? 0.0d : b;
}
*/

/*
double lerp(double motorPower)
{
  double absMotorPower = abs(motorPower);
  if (absMotorPower > LINEAR_MIN_POWER)
    return motorPower;

  double lerp = quadraticLerp0(absMotorPower / LINEAR_MIN_POWER,
      LINEAR_MIN_POWER, LINEAR_MIN_POWER - (0.1d * LINEAR_MIN_POWER));
  return motorPower >= 0.0d ? lerp : -lerp;
}
*/

/*
double snap(double motorPower)
{
  double absMotorPower = abs(motorPower);
  if (absMotorPower < LINEAR_MIN_THRESHOLD)
    return 0.0d;

  double power = LINEAR_MIN_POWER + absMotorPower;
  return motorPower < 0.0d ? -power : power;
}
*/

/*
double pad(double motorPower)
{
  if (motorPower > 0.0d)
    return LINEAR_MIN_POWER + motorPower;
  else if (motorPower < 0.0d)
    return -LINEAR_MIN_POWER + motorPower;

  return 0.0d;
}
*/
