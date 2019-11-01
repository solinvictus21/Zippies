
#include <SPI.h>
#include "Zippy.h"
#include "ZippyRoutine.h"
#include "ZippyConfig.h"
#include "paths/Turn.h"
#include "paths/Move.h"
#include "paths/Arc.h"
#include "paths/CompositePath.h"
#include "paths/ZPathPlanner.h"

#define WHEEL_Kp                               240.00d
#define WHEEL_Ki                                 0.00d
#define WHEEL_Kd                                13.40d
#define WHEEL_MAX_POWER                      30000.00d

// #define PID_INTERVAL                            17
#define PID_INTERVAL                            17

//the distince in mm within which is to be considered "at the target" for the purpose of terminating movement
#define START_LINEAR_EPSILON_2                       25.00d
#define FOLLOW_EPSILON                               10.00d
#define STOP_LINEAR_EPSILON_2                       144.00d
//the delta angle within which is to be considered "pointing at the desired orientation" (3 degrees)
#define ANGULAR_EPSILON                          0.052359877559830d

const double WHEEL_RADIAL_OFFSET = WHEEL_OFFSET_X;
// const double WHEEL_RADIAL_OFFSET = sqrt(sq(WHEEL_OFFSET_X) + sq(WHEEL_OFFSET_Y));
// const double WHEEL_SIN = WHEEL_OFFSET_X / WHEEL_RADIAL_OFFSET;

Zippy::Zippy()
  : leftWheel(-WHEEL_RADIAL_OFFSET,
        WHEEL_Kp, WHEEL_Ki, WHEEL_Kd, WHEEL_MAX_POWER, PID_INTERVAL),
    rightWheel(WHEEL_RADIAL_OFFSET,
        WHEEL_Kp, WHEEL_Ki, WHEEL_Kd, WHEEL_MAX_POWER, PID_INTERVAL)
{
}

void Zippy::start()
{
}

void Zippy::startErrorCapture()
{
  errorCaptureEnabled = true;
  errorMin = 1000000.0d;
  errorAccumulator = 0.0d;
  errorMax = 0.0d;
  errorCounter = 0;
}

void Zippy::setInputs(const KMatrix2* cp, const KMatrix2* cv)
{
  currentPosition.set(cp);
  currentVelocity.set(cv);

  if (errorCaptureEnabled &&
      (currentMovementState == MovementState::Moving || currentMovementState == MovementState::Turning) &&
      targetVelocity.position.getD2() >= 1.0d)
    captureError();
}

void Zippy::captureError()
{
  //only capture data for statistically significant target velocities
  double error = sqrt(sq(currentVelocity.position.getX() - targetVelocity.position.getX()) +
      sq(currentVelocity.position.getY() - targetVelocity.position.getY())) /
      targetVelocity.position.getD();

  errorAccumulator += error;
  errorCounter++;
  errorMin = min(errorMin, error);
  errorMax = max(errorMax, error);
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
  processInputs();

  //check if we can downgrade our current movement state
  targetVelocity.set(&targetPosition);
  targetVelocity.unconcat(&currentPosition);
  switch (currentMovementState) {
    case MovementState::PreparingToMove:
      if (targetVelocity.position.getD2() < START_LINEAR_EPSILON_2) {
        if (!targetPositionUpdated) {
          leftWheel.start();
          rightWheel.start();
          currentMovementState = MovementState::Turning;
        }
      }
      else {
        leftWheel.start();
        rightWheel.start();
        currentMovementState = MovementState::Moving;
      }
      break;

    case MovementState::Moving:
      if (!targetPositionUpdated) {
        if (targetVelocity.position.getD2() < STOP_LINEAR_EPSILON_2)
            currentMovementState = MovementState::Turning;
      }
      else
        targetVelocity.position.setD(max(targetVelocity.position.getD() - FOLLOW_EPSILON, 0.0d));
      break;

    case MovementState::Turning:
      if (!targetOrientationUpdated && abs(targetVelocity.orientation.get()) < ANGULAR_EPSILON) {
        leftWheel.stop();
        rightWheel.stop();
        motors.stopMotors();
        currentMovementState = MovementState::Stopped;
      }
      break;
  }
  targetPositionUpdated = false;
  targetOrientationUpdated = false;

  switch (currentMovementState) {
    case MovementState::Moving:
      executeMove();
      break;

    case MovementState::Turning:
      executeTurn();
      break;
  }
}

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

void Zippy::executeMove()
{
  double targetAtan = targetVelocity.position.atan();
  if (abs(targetAtan) < DOUBLE_EPSILON) {
    //just drive straight
    double velocity = targetVelocity.position.getY();
    leftWheel.moveStraight(velocity);
    rightWheel.moveStraight(velocity);
  }
  else {
    //simple arc
    double subtendedAngle = 2.0d * targetAtan;
    double radius = targetVelocity.position.getD() / (2.0 * sin(targetVelocity.position.atan2()));
    leftWheel.moveArc(radius, subtendedAngle);
    rightWheel.moveArc(radius, subtendedAngle);
  }
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

void Zippy::executeTurn()
{
  double deltaOrientation = targetVelocity.orientation.get();
  leftWheel.turn(deltaOrientation);
  rightWheel.turn(deltaOrientation);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

void Zippy::stopErrorCapture()
{
  errorCaptureEnabled = false;
}

//stopped when the signal from the lighthouse is lost
void Zippy::stop()
{
  motors.stopMotors();
}
