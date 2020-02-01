
#include "zippies/controllers/PathFollowingController.h"
#include <Arduino.h>

#define TARGET_EPSILON                                0.30d

#define LINEAR_EPSILON                               10.00d
const double MIN_LINEAR_EPSILON = LINEAR_EPSILON * (1.0d - TARGET_EPSILON);
const double MAX_LINEAR_EPSILON = LINEAR_EPSILON * (1.0d + TARGET_EPSILON);

// #define ANGULAR_EPSILON                               0.052359877559830d  //3 degrees
// #define ANGULAR_EPSILON                               0.087266462599716d  //5 degrees
// #define ANGULAR_EPSILON                               0.104719755119660d  //6 degrees
// #define ANGULAR_EPSILON                               0.139626340159546d  //8 degrees
#define ANGULAR_EPSILON                               0.174532925199433d  //10 degrees
// #define ANGULAR_EPSILON                               0.261799387799149d  //15 degrees
// #define ANGULAR_EPSILON                               0.069813170079773d  //4 degrees
const double MIN_ANGULAR_EPSILON = ANGULAR_EPSILON * (1.0d - TARGET_EPSILON);
const double MAX_ANGULAR_EPSILON = ANGULAR_EPSILON * (1.0d + TARGET_EPSILON);

void PathFollowingController::followPath(
    const KMatrix2* currentPosition,
    const KMatrix2* targetPosition,
    MovementState targetMovementState)
{
  currentMovement.set(targetPosition);
  currentMovement.unconcat(currentPosition);
  if (targetMovementState == MovementState::Moving)
    calculateRelativeBiArcKnot(&currentMovement);

  if (targetMovementState > currentMovementState) {
    if (currentMovementState == MovementState::Stopped)
      zippy.start();
    currentMovementState = targetMovementState;
  }
  else if (targetMovementState < currentMovementState) {
    switch (currentMovementState) {
      case MovementState::Moving:
        if (currentMovement.position.getD() < MIN_LINEAR_EPSILON)
          currentMovementState = MovementState::Turning;
        break;

      case MovementState::Turning:
        if (abs(currentMovement.orientation.get()) < MIN_ANGULAR_EPSILON) {
          zippy.stop();
          currentMovementState = MovementState::Stopped;
        }
        break;
    }
  }

  //now execute a move if needed
  switch (currentMovementState) {
    case MovementState::Moving:
      // zippy.setVelocity(currentMovement.position.getD(), currentMovement.position.atan());
      zippy.move(&currentMovement);
      break;

    case MovementState::Turning:
      // zippy.setVelocity(0.0d, currentMovement.orientation.get());
      zippy.turn(currentMovement.orientation.get());
      break;
  }
}

void PathFollowingController::stop()
{
  zippy.stop();
  currentMovementState = MovementState::Stopped;
}
