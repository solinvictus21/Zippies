
#include "zippies/controllers/RoutineController.h"
#include "zippies/ZippyRoutine.h"
#include "zippies/config/ZippyDefaultRoutine.h"

#define INITIAL_MOVE_VELOCITY          200.0d
#define INITIAL_MOVE_TIMING           8000

PathDefinition moveIntoPlace[]
{
  { PathDefinitionType::Arc ,        0.0d               },
  { PathDefinitionType::Arc ,        0.0d               },
};

RoutineDefinition moveIntoPlaceRoutine[]
{
  //bi-arc move into place
  {     0, 0.20d, 0.10d, sizeof(moveIntoPlace) / sizeof(PathDefinition), moveIntoPlace, 0 },
  //initial pause
  {     0, 0.00d, 0.00d, 0,          NULL, 0 },
};
const int moveIntoPlaceRoutineLength = sizeof(moveIntoPlaceRoutine) / sizeof(RoutineDefinition);

RoutineController::RoutineController(SensorFusor* s)
  : sensors(s)
{
  getZippyDefaultRoutines(
      &defaultRoutinesStartPosition,
      &defaultRoutines,
      &defaultRoutinesCount);
}

void RoutineController::start(unsigned long currentTime)
{
  sensors->syncWithPreamble();
  // reverseRoutine(defaultRoutines, defaultRoutinesCount);
  executionState = RoutineExecutionState::PreSyncingWithPreamble;
}

void RoutineController::createRelativePathDefinition(const KVector2* relativeMovement, PathDefinition* pathDefinition)
{
  double direction = relativeMovement->atan();
  if (direction == 0.0d) {
    pathDefinition->type = PathDefinitionType::Move;
    pathDefinition->params.p1 = relativeMovement->getY();
  }
  else if (relativeMovement->getD2() == 0.0d) {
    pathDefinition->type = PathDefinitionType::Turn;
    pathDefinition->params.p1 = direction;
  }
  else {
    pathDefinition->type = PathDefinitionType::Arc;
    pathDefinition->params.p1 = relativeMovement->getD() / (2.0d * sin(relativeMovement->atan2()));
    pathDefinition->params.p2 = 2.0d * direction;
  }
}

void RoutineController::planMoveIntoPlace(const KMatrix2* currentPosition, const KMatrix2* startPosition)
{
  //calculate our path to the initial starting position
  KMatrix2 movement2(startPosition);
  movement2.unconcat(currentPosition);
  KMatrix2 movement1;
  calculateRelativeBiArcKnot(&movement2, &movement1);
  movement2.unconcat(&movement1);
  createRelativePathDefinition(&movement1.position, &moveIntoPlace[0]);
  createRelativePathDefinition(&movement2.position, &moveIntoPlace[1]);

  double pathDistance =
      getPathSegmentLength(&moveIntoPlace[0]) +
      getPathSegmentLength(&moveIntoPlace[1]);
  moveIntoPlaceRoutine[0].timing = min(1000.0d * (pathDistance / INITIAL_MOVE_VELOCITY), INITIAL_MOVE_TIMING);
  moveIntoPlaceRoutine[1].timing = INITIAL_MOVE_TIMING - moveIntoPlaceRoutine[0].timing;
}

void RoutineController::loop(unsigned long currentTime)
{
  switch (executionState) {

    case RoutineExecutionState::PreSyncingWithPreamble:
      if (sensors->foundPreamble()) {
        const KMatrix2* currentPosition = sensors->getPosition();
        // SerialUSB.println("Found preamble. Moving into position.");
        // planMoveIntoPlace(currentPosition, &DEFAULT_START_POSITION);
        planMoveIntoPlace(currentPosition, &defaultRoutinesStartPosition);
        //provide the move-into-place routine to our routing controller
        routine.setRoutineSegments(currentPosition, moveIntoPlaceRoutine, moveIntoPlaceRoutineLength);
        routine.start(currentTime);
        executionState = RoutineExecutionState::MovingIntoPlace;
      }
      break;

    case RoutineExecutionState::MovingIntoPlace:
      routine.loop(currentTime);
      pathFollowingController.followPath(
          sensors->getPosition(),
          routine.getTargetPosition(),
          routine.getTargetMovementState());

      if (routine.isRoutineCompleted()) {
        routine.setRoutineSegments(&defaultRoutinesStartPosition, defaultRoutines, defaultRoutinesCount);
        routine.start(currentTime);
        executionState = RoutineExecutionState::Executing;
      }
      break;

    case RoutineExecutionState::Executing:
      routine.loop(currentTime);
      pathFollowingController.followPath(
          sensors->getPosition(),
          routine.getTargetPosition(),
          routine.getTargetMovementState());

      if (routine.isRoutineCompleted()) {
        //sync with preamble again before start of each routine execution
        sensors->syncWithPreamble();
        executionState = RoutineExecutionState::PostSyncingWithPreamble;
      }
      break;

    case RoutineExecutionState::PostSyncingWithPreamble:
      pathFollowingController.followPath(
          sensors->getPosition(),
          routine.getTargetPosition(),
          routine.getTargetMovementState());

      if (sensors->foundPreamble()) {
        // reverseRoutine(defaultRoutines, defaultRoutinesCount);
        routine.start(currentTime);
        executionState = RoutineExecutionState::Executing;
      }
      break;
  }
}

void RoutineController::stop()
{
  pathFollowingController.stop();
}
