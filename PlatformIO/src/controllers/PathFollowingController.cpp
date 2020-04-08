
#include "zippies/controllers/PathFollowingController.h"
#include <Arduino.h>

#define LINEAR_EPSILON                               10.00d  //1cm
// #define LINEAR_EPSILON                               15.00d  //1.5cm
// #define LINEAR_EPSILON                               20.00d  //2cm
// #define LINEAR_EPSILON                               30.00d  //3cm

// #define ANGULAR_EPSILON                               0.052359877559830d  //3 degrees
#define ANGULAR_EPSILON                               0.069813170079773d  //4 degrees
// #define ANGULAR_EPSILON                               0.087266462599716d  //5 degrees
// #define ANGULAR_EPSILON                               0.104719755119660d  //6 degrees
// #define ANGULAR_EPSILON                               0.139626340159546d  //8 degrees
// #define ANGULAR_EPSILON                               0.174532925199433d  //10 degrees
// #define ANGULAR_EPSILON                               0.261799387799149d  //15 degrees

#define MAX_STATE_DOWNGRADE_ITERATIONS               30
// #define MAX_STATE_DOWNGRADE_ITERATIONS              120

double pad(double value, double epsilon)
{
  if (value == 0.0d)
    return 0.0d;

  return value < 0.0d
    ? min(value, -epsilon)
    : max(value, epsilon);
}

void PathFollowingController::followPath(
    const KMatrix2* currentPosition,
    const KMatrix2* targetPosition,
    MovementState targetMovementState)
{
  currentMovement.set(targetPosition);
  currentMovement.unconcat(currentPosition);

  switch (targetMovementState) {
    case MovementState::Moving:
      if (currentMovementState == MovementState::Stopped)
        zippy.start();

      stateDowngradeIterationCount = 0;
      currentMovementState = MovementState::Moving;
      break;

    case MovementState::Turning:
      if (currentMovementState == MovementState::Moving) {
        //state downgrade to turning pending
        stateDowngradeIterationCount++;
        if (currentMovement.position.getD() < LINEAR_EPSILON ||
            stateDowngradeIterationCount >= MAX_STATE_DOWNGRADE_ITERATIONS)
        {
          //state downgrade from moving to turning
          stateDowngradeIterationCount = 0;
          currentMovementState = MovementState::Turning;
        }
      }
      else if (currentMovementState == MovementState::Stopped) {
        //state upgrade from stopped to turning
        zippy.start();
        stateDowngradeIterationCount = 0;
        currentMovementState = MovementState::Turning;
      }
      break;

    case MovementState::Stopped:
      if (currentMovementState == MovementState::Moving) {
        //state downgrade to stopped pending
        stateDowngradeIterationCount++;
        if (currentMovement.position.getD() < LINEAR_EPSILON ||
            stateDowngradeIterationCount >= MAX_STATE_DOWNGRADE_ITERATIONS)
        {
          //state downgrade from moving to turning
          stateDowngradeIterationCount = 0;
          currentMovementState = MovementState::Turning;
        }
      }

      if (currentMovementState == MovementState::Turning) {
        stateDowngradeIterationCount++;
        if (//abs(currentMovement.orientation.get()) < ANGULAR_EPSILON ||
            stateDowngradeIterationCount >= MAX_STATE_DOWNGRADE_ITERATIONS)
        {
          //state downgrade from turning to stopped
          stateDowngradeIterationCount = 0;
          zippy.stop();
          currentMovementState = MovementState::Stopped;
        }
      }
      break;
  }

  //now execute a move if needed
  switch (currentMovementState) {
    case MovementState::Moving:
      /*
      if (stateDowngradeIterationCount)
        currentMovement.position.multiply(0.5d + (0.3d * ((double)stateDowngradeIterationCount) / ((double)MAX_STATE_DOWNGRADE_ITERATIONS)));
      else
        currentMovement.position.multiply(0.5d);
      zippy.move(&currentMovement);
      */
      move();
      break;

    case MovementState::Turning:
      zippy.turn(pad(currentMovement.orientation.get(), 2.0d * ANGULAR_EPSILON));
      break;
  }
}

void PathFollowingController::moveDirect()
{
  if (currentMovement.position.getX() == 0.0d) {
    zippy.moveLinear(currentMovement.position.getY());
    return;
  }

  zippy.moveArc(
      currentMovement.position.getD2() / (2.0d * currentMovement.position.getX()),
      2.0d * currentMovement.position.atan());
}

void PathFollowingController::move()
{
  double factor;
  if (stateDowngradeIterationCount) {
    factor = 0.4d + (0.4d * ((double)stateDowngradeIterationCount) / ((double)MAX_STATE_DOWNGRADE_ITERATIONS));
    currentMovement.position.multiply(factor);
    moveDirect();
    return;
    // factor = 0.5d + (0.3d * ((double)stateDowngradeIterationCount) / ((double)MAX_STATE_DOWNGRADE_ITERATIONS));
  }
  // else {
    //adjust the movement depending on how well we will arrive at our target orientation
    double targetDirection = currentMovement.position.atan();
    double deltaOrientationAtTarget = subtractAngles(
        currentMovement.orientation.get(),
        2.0d * targetDirection);
    // factor = cos(deltaOrientationAtTarget / 2.0d);
    factor = 0.1d + (0.5d * cos(deltaOrientationAtTarget / 2.0d));
  // }

  // double factor = 0.5d;
  if (currentMovement.position.getX() == 0.0d) {
    zippy.moveLinear(currentMovement.position.getY() * factor);
    return;
  }

  double movementRadius = currentMovement.position.getD2() / (2.0d * currentMovement.position.getX());
  movementRadius *= factor;
  KVector2 radiusToTarget = KVector2(
      currentMovement.position.getY(),
      movementRadius - currentMovement.position.getX());
  double completeTurnToTarget = radiusToTarget.atan2();
  double movementTheta = completeTurnToTarget > 0.0d
      ? completeTurnToTarget - acos(movementRadius / radiusToTarget.getD())
      : completeTurnToTarget + acos(movementRadius / radiusToTarget.getD());
  zippy.moveArc(movementRadius, movementTheta);
}

void PathFollowingController::stop()
{
  zippy.stop();
  currentMovementState = MovementState::Stopped;
}
