
#include <SPI.h>
#include "Zippy.h"
#include "ZippyRoutine.h"
#include "ZippyConfig.h"
#include "paths/Turn.h"
#include "paths/Move.h"
#include "paths/Arc.h"
#include "paths/CompositePath.h"
#include "paths/ZPathPlanner.h"

#define WHEEL_MAX_POWER                      30000.00d

// #define PID_INTERVAL                            17
#define PID_INTERVAL                            17

//the distince in mm within which is to be considered "at the target" for the purpose of terminating movement
#define START_LINEAR_EPSILON_2                      100.00d
#define FOLLOW_FACTOR                                 0.60d
// #define STOP_LINEAR_EPSILON_2                       225.00d
#define STOP_LINEAR_EPSILON_2                       100.00d
//5 degrees
// #define MIN_ANGULAR_EPSILON                           0.087266462599716d
//10 degrees
#define MIN_ANGULAR_EPSILON                           0.174532925199433d
//15 degrees
// #define MIN_ANGULAR_EPSILON                           0.261799387799149d
//20 degrees
// #define MIN_ANGULAR_EPSILON                           0.349065850398866d
//40 degrees
// #define MIN_ANGULAR_EPSILON                           0.698131700797732d
//the delta angle within which is to be considered "pointing at the desired orientation" (3 degrees)
// #define STOP_ANGULAR_EPSILON                          0.052359877559830d
//5 degrees
// #define STOP_ANGULAR_EPSILON                          0.087266462599716d
//15 degrees
#define STOP_ANGULAR_EPSILON                           0.261799387799149d

const double WHEEL_RADIAL_OFFSET = WHEEL_OFFSET_X;
// const double WHEEL_RADIAL_OFFSET = sqrt(sq(WHEEL_OFFSET_X) + sq(WHEEL_OFFSET_Y));
// const double WHEEL_SIN = WHEEL_OFFSET_X / WHEEL_RADIAL_OFFSET;

Zippy::Zippy()
  : leftWheel(-WHEEL_RADIAL_OFFSET,
        DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, WHEEL_MAX_POWER, PID_INTERVAL),
    rightWheel(WHEEL_RADIAL_OFFSET,
        DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, WHEEL_MAX_POWER, PID_INTERVAL)
{
}

void Zippy::start()
{
}

void Zippy::startErrorCapture()
{
  errorCaptureEnabled = true;
  errorM = 0.0d;
  errorS = 0.0d;
  errorCounter = 0;
}

void Zippy::setCurrentPosition(const KMatrix2* p)
{
  currentPosition.set(p);

  //only capture error statistics while moving
  if (errorCaptureEnabled && currentMovementState == MovementState::Moving)
    captureError();
}

void Zippy::captureError()
{
  //calculate error based on how close we came to our target position
  double error = sqrt(
      sq(currentPosition.position.getX() - targetPosition.position.getX()) +
      sq(currentPosition.position.getY() - targetPosition.position.getY()));

  //keep a running standard deviation
  //base on info derived from https://www.johndcook.com/blog/standard_deviation
  errorCounter++;
  double nextErrorM = errorM + ((error - errorM) / ((double)errorCounter));
  errorS = errorS + ((error - errorM) * (error - nextErrorM));
  errorM = nextErrorM;
}

double Zippy::getStandardDeviation() const
{
  return errorCounter > 1 ? sqrt(errorS / ((double)(errorCounter-1))) : 0.0d;
}

void Zippy::setTargetPosition(const KMatrix2* tp)
{
  targetPosition.set(tp);
  targetPositionUpdated = true;
  targetOrientationUpdated = true;
  if (currentMovementState == MovementState::Stopped)
    currentMovementState = MovementState::PreparingToMove;
  else if (currentMovementState == MovementState::Turning)
    currentMovementState = MovementState::Moving;
}

void Zippy::setTargetOrientation(const KRotation2* r)
{
  targetPosition.orientation.set(r);
  targetOrientationUpdated = true;
  if (currentMovementState == MovementState::Stopped) {
    leftWheel.start();
    rightWheel.start();
    currentMovementState = MovementState::Turning;
  }
}

void Zippy::loop()
{
  //check if we can downgrade our current movement state
  targetVelocity.set(&targetPosition);
  targetVelocity.unconcat(&currentPosition);
  switch (currentMovementState) {
    case MovementState::PreparingToMove:
      if (!targetPositionUpdated) {
        leftWheel.start();
        rightWheel.start();
        if (targetVelocity.position.getD2() < STOP_LINEAR_EPSILON_2)
          currentMovementState = MovementState::Turning;
        else
          currentMovementState = MovementState::Moving;
      }
      else if (targetVelocity.position.getD2() >= START_LINEAR_EPSILON_2) {
        leftWheel.start();
        rightWheel.start();
        currentMovementState = MovementState::Moving;
      }
      break;

    case MovementState::Moving:
      if (!targetPositionUpdated && targetVelocity.position.getD2() < STOP_LINEAR_EPSILON_2)
        currentMovementState = MovementState::Turning;
      break;

    case MovementState::Turning:
      if (!targetOrientationUpdated && abs(targetVelocity.orientation.get()) < STOP_ANGULAR_EPSILON)
        stop();
      break;
  }

  //now execute a move if needed
  switch (currentMovementState) {
    case MovementState::Moving:
      executeMove();
      break;

    case MovementState::Turning:
      executeTurn();
      break;
  }

  targetPositionUpdated = false;
  targetOrientationUpdated = false;
}

/*
void Zippy::processInputs()
{
  //process our PID inputs
  double currentAtan = currentVelocity.position.atan();
  if (abs(currentAtan) < DOUBLE_EPSILON) {
    //asymptote; direction is straight forward or straight backward, which would cause the
    //curve radius to be NaN / infinite or, at the very least, create a radius that would
    //induce far too much Abbe error into the arc calculation
    double linearVelocity = currentVelocity.position.getY();
    leftWheel.setInputStraight(linearVelocity);
    rightWheel.setInputStraight(linearVelocity);
  }
  else {
    //simple arc
    double subtendedAngle = 2.0d * currentAtan;
    double radius = currentVelocity.position.getD() / (2.0d * sin(currentVelocity.position.atan2()));
    leftWheel.setInputArc(radius, subtendedAngle);
    rightWheel.setInputArc(radius, subtendedAngle);
  }
}
*/

void Zippy::executeMove()
{
  double targetAtan = targetVelocity.position.atan();
  if (abs(targetAtan) < DOUBLE_EPSILON) {
    executeLinearMove();
    return;
  }

  //simple arc
  double subtendedAngle = 2.0d * targetAtan;
  double radius = targetVelocity.position.getD() / (2.0 * sin(targetVelocity.position.atan2()));

  double endingAngle = addAngles(currentPosition.orientation.get(), subtendedAngle);
  if (abs(subtractAngles(targetPosition.orientation.get(), endingAngle)) >= M_PI_2) {
    //in this situation, moving toward the point faces us away from the target
    subtendedAngle = -subtendedAngle;
    radius = -radius;
  }

  // /*
  //follow behind the target point while actively Moving
  if (targetPositionUpdated) {
    subtendedAngle *= FOLLOW_FACTOR;

    //when tracking error for the purpose of PID tuning, it's important to update the target position to accurately track error
    if (errorCaptureEnabled) {
      double newDistance = abs(radius * subtendedAngle);
      double newTargetAtan = subtendedAngle / 2.0d;
      targetVelocity.set(
        newDistance * sin(newTargetAtan),
        newDistance * cos(newTargetAtan),
        newTargetAtan);
      targetPosition.set(&targetVelocity);
      targetPosition.concat(&currentPosition);
    }
  }
  // */

  leftWheel.moveArc(radius, subtendedAngle);
  rightWheel.moveArc(radius, subtendedAngle);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

void Zippy::executeLinearMove()
{
  //just drive straight
  double velocity = targetVelocity.position.getY();

  //follow behind the target point while actively Moving
  if (targetPositionUpdated)
    velocity *= FOLLOW_FACTOR;

  //when tracking error for the purpose of PID tuning, it's important to update the target position
  if (errorCaptureEnabled) {
    targetVelocity.set(0.0d, velocity, 0.0d);
    targetPosition.set(&targetVelocity);
    targetPosition.concat(&currentPosition);
  }

  leftWheel.moveStraight(velocity);
  rightWheel.moveStraight(velocity);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

void Zippy::executeTurn()
{
  double deltaOrientation = targetVelocity.orientation.get();
  if (abs(deltaOrientation) < MIN_ANGULAR_EPSILON)
    deltaOrientation = deltaOrientation < 0.0d ? -MIN_ANGULAR_EPSILON : MIN_ANGULAR_EPSILON;
  leftWheel.turn(deltaOrientation);
  rightWheel.turn(deltaOrientation);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

//stopped when the signal from the lighthouse is lost
void Zippy::stop()
{
  targetVelocity.reset();
  leftWheel.stop();
  rightWheel.stop();
  motors.stopMotors();
  currentMovementState = MovementState::Stopped;
}

void Zippy::stopErrorCapture()
{
  errorCaptureEnabled = false;
}
