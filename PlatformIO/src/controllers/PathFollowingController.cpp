
#include "zippies/controllers/PathFollowingController.h"
#include <Arduino.h>

// #define LINEAR_EPSILON                               10.00  //1cm
#define LINEAR_EPSILON                               15.00  //1.5cm
// #define LINEAR_EPSILON                               20.00  //2cm
// #define LINEAR_EPSILON                               30.00  //3cm

// #define ANGULAR_EPSILON                               0.052359877559830  //3 degrees
#define ANGULAR_EPSILON                               0.069813170079773  //4 degrees
// #define ANGULAR_EPSILON                               0.087266462599716  //5 degrees
// #define ANGULAR_EPSILON                               0.104719755119660  //6 degrees
// #define ANGULAR_EPSILON                               0.139626340159546  //8 degrees
// #define ANGULAR_EPSILON                               0.174532925199433  //10 degrees
// #define ANGULAR_EPSILON                               0.261799387799149  //15 degrees

#define MAX_STATE_DOWNGRADE_ITERATIONS               30
// #define MAX_STATE_DOWNGRADE_ITERATIONS              120

double pad(double value, double epsilon)
{
  if (value == 0.0)
    return 0.0;

  return value < 0.0
    ? min(value, -epsilon)
    : max(value, epsilon);
}

void PathFollowingController::followPath(
    const ZMatrix2* currentPosition,
    const ZMatrix2* targetPosition,
    MovementState targetMovementState)
{
  currentMovement.set(targetPosition);
  currentMovement.unconcat(currentPosition);

  switch (targetMovementState) {
    case MovementState::Moving:
      executeMove();
      break;

    case MovementState::Turning:
      executeTurn();
      break;

    case MovementState::Stopped:
      executeStop();
      break;
  }
}

void PathFollowingController::followPath(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
  currentMovement.set(
    targetPosition->getX(), targetPosition->getY(),
    targetVelocity->atan2());
  currentMovement.unconcat(currentPosition);
  executeMove();
}

void PathFollowingController::stop()
{
  zippy.stop();
  currentMovementState = MovementState::Stopped;
}

void PathFollowingController::executeMove()
{
  switch (currentMovementState) {
    case MovementState::Turning:
      stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
      currentMovementState = MovementState::Moving;
      break;

    case MovementState::Stopped:
      zippy.start();
      stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
      currentMovementState = MovementState::Moving;
      break;
  }

  //adjust the movement depending on how well we will arrive at our target orientation
  double targetDirection = currentMovement.position.atan();
  double deltaOrientationAtTarget = subtractAngles(
      currentMovement.orientation.get(),
      2.0 * targetDirection);
  // factor = cos(deltaOrientationAtTarget / 2.0d);
  double factor = 0.1 + (0.5 * cos(deltaOrientationAtTarget / 2.0));

  // double factor = 0.5d;
  if (currentMovement.position.getX() == 0.0) {
    zippy.moveLinear(currentMovement.position.getY() * factor);
    return;
  }

  double movementRadius = currentMovement.position.getD2() / (2.0 * currentMovement.position.getX());
  movementRadius *= factor;
  ZVector2 radiusToTarget = ZVector2(
      currentMovement.position.getY(),
      movementRadius - currentMovement.position.getX());
  double completeTurnToTarget = radiusToTarget.atan2();
  double movementTheta = completeTurnToTarget > 0.0
      ? completeTurnToTarget - acos(movementRadius / radiusToTarget.getD())
      : completeTurnToTarget + acos(movementRadius / radiusToTarget.getD());
  zippy.moveArc(movementRadius, movementTheta);
}

void PathFollowingController::executeTurn()
{
  switch (currentMovementState) {
    case MovementState::Moving:
      if (currentMovement.position.getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
      {
        stateDowngradeCounter--;
        moveDirect();
        return;
      }

      stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
      currentMovementState = MovementState::Turning;
      break;

    case MovementState::Stopped:
      zippy.start();
      stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
      currentMovementState = MovementState::Turning;
      break;
  }

  zippy.turn(pad(currentMovement.orientation.get(), 2.0 * ANGULAR_EPSILON));
}

void PathFollowingController::executeStop()
{
  switch (currentMovementState) {
    case MovementState::Moving:
      if (currentMovement.position.getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
      {
        stateDowngradeCounter--;
        moveDirect();
      }
      else
      {
        stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
        currentMovementState = MovementState::Turning;
      }
      return;

    case MovementState::Turning:
      if (abs(currentMovement.orientation.get()) > ANGULAR_EPSILON &&
          stateDowngradeCounter)
      {
        stateDowngradeCounter--;
        zippy.turn(pad(currentMovement.orientation.get(), 2.0 * ANGULAR_EPSILON));
        return;
      }

      currentMovementState = MovementState::Stopped;
      break;
  }

  stop();
}

void PathFollowingController::moveDirect()
{
  double factor = 1.0 - (0.6 * ((double)stateDowngradeCounter) / ((double)MAX_STATE_DOWNGRADE_ITERATIONS));
  currentMovement.position.multiply(factor);
  if (currentMovement.position.getX() == 0.0) {
    zippy.moveLinear(currentMovement.position.getY());
    return;
  }

  zippy.moveArc(
      currentMovement.position.getD2() / (2.0 * currentMovement.position.getX()),
      2.0 * currentMovement.position.atan());
}

void PathFollowingController::stopPath(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
  currentMovement.set(
    targetPosition->getX(), targetPosition->getY(),
    targetVelocity->atan2());
  currentMovement.unconcat(currentPosition);
  executeStop();
}
