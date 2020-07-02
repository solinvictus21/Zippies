
#include "zippies/controllers/RoutineController.h"
#include "zippies/ZippyRoutine.h"
#include "zippies/config/ZippyDefaultRoutine.h"

#define INITIAL_MOVE_VELOCITY           200.0
#define INITIAL_MOVE_TIMING            8000

PathDefinition moveIntoPlace[]
{
  { PathDefinitionType::Arc ,        0.0               },
  { PathDefinitionType::Arc ,        0.0               },
};

RoutineDefinition moveIntoPlaceRoutine[]
{
  //bi-arc move into place
  {     0, 0.20, 0.10, sizeof(moveIntoPlace) / sizeof(PathDefinition), moveIntoPlace, 0 },
  //initial pause
  {     0, 0.00, 0.00, 0,          NULL, 0 },
};
const int moveIntoPlaceRoutineLength = sizeof(moveIntoPlaceRoutine) / sizeof(RoutineDefinition);

RoutineController::RoutineController(SensorFusor* s)
  : sensors(s),
    path(zippyPath, zippyPathCount)
{
    getZippyDefaultRoutines(
        &defaultRoutinesStartPosition,
        &defaultRoutines,
        &defaultRoutinesCount);
    
    /*
    SerialUSB.println("Constructed routine controller.");
    path.start(0);
    for (unsigned long i = 0; i <= 16000; i += 1000)
    {
        path.interpolate(i);
        SerialUSB.print("positions/tangents ");
        SerialUSB.println(i);
        path.getTargetPosition()->printDebug();
        path.getTargetVelocity()->printDebug();
        SerialUSB.println(180.0 * path.getTargetVelocity()->atan2() / M_PI, 2);
    }
    */
}

void RoutineController::start(unsigned long currentTime)
{
  sensors->syncWithPreamble();
  // reverseRoutine(defaultRoutines, defaultRoutinesCount);
  executionState = RoutineExecutionState::PreSyncingWithPreamble;
}

void RoutineController::createRelativePathDefinition(const ZVector2* relativeMovement, PathDefinition* pathDefinition)
{
  double direction = relativeMovement->atan();
  if (direction == 0.0) {
    pathDefinition->type = PathDefinitionType::Move;
    pathDefinition->params.p1 = relativeMovement->getY();
  }
  else if (relativeMovement->getD2() == 0.0) {
    pathDefinition->type = PathDefinitionType::Turn;
    pathDefinition->params.p1 = direction;
  }
  else {
    pathDefinition->type = PathDefinitionType::Arc;
    pathDefinition->params.p1 = relativeMovement->getD() / (2.0 * sin(relativeMovement->atan2()));
    pathDefinition->params.p2 = 2.0 * direction;
  }
}

void RoutineController::planMoveIntoPlace(const ZMatrix2* currentPosition, const ZMatrix2* startPosition)
{
  //calculate our path to the initial starting position
  ZMatrix2 movement2(startPosition);
  movement2.unconcat(currentPosition);
  ZMatrix2 movement1;
  calculateRelativeBiArcKnot(&movement2, &movement1);
  movement2.unconcat(&movement1);
  createRelativePathDefinition(&movement1.position, &moveIntoPlace[0]);
  createRelativePathDefinition(&movement2.position, &moveIntoPlace[1]);

  double pathDistance =
      getPathSegmentLength(&moveIntoPlace[0]) +
      getPathSegmentLength(&moveIntoPlace[1]);
  moveIntoPlaceRoutine[0].timing = min(1000.0 * (pathDistance / INITIAL_MOVE_VELOCITY), INITIAL_MOVE_TIMING);
  moveIntoPlaceRoutine[1].timing = INITIAL_MOVE_TIMING - moveIntoPlaceRoutine[0].timing;
}

void RoutineController::loop(unsigned long currentTime)
{
  switch (executionState) {

    case RoutineExecutionState::PreSyncingWithPreamble:
      if (sensors->foundPreamble()) {
        const ZMatrix2* currentPosition = sensors->getPosition();
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
        path.start(currentTime);
        executionState = RoutineExecutionState::Executing;
      }
      break;

    case RoutineExecutionState::Executing:
      path.interpolate(currentTime);
      pathFollowingController.followPath(
        sensors->getPosition(),
        path.getTargetPosition(),
        path.getTargetVelocity());
      
      if (path.isCompleted()) {
        //sync with preamble again before start of each routine execution
        sensors->syncWithPreamble();
        executionState = RoutineExecutionState::PostSyncingWithPreamble;
      }
      break;

    case RoutineExecutionState::PostSyncingWithPreamble:
      pathFollowingController.stopPath(
        sensors->getPosition(),
        path.getTargetPosition(),
        path.getTargetVelocity());

      if (sensors->foundPreamble()) {
        path.start(currentTime);
        executionState = RoutineExecutionState::Executing;
      }
      break;
  }
}

void RoutineController::stop()
{
  pathFollowingController.stop();
}
