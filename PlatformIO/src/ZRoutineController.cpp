
#include "ZippyConfig.h"
#include "ZRoutineController.h"
#include "paths/Turn.h"
#include "paths/Move.h"
#include "paths/Arc.h"
#include "paths/CompositePath.h"
#include "paths/ZPathPlanner.h"
#include "lighthouse/KVector3.h"

const ZPath* convertToPath(const KMatrix2* start, const Command* command);

ZRoutineController::ZRoutineController(Zippy* z)
  : zippy(z),
    routine(ROUTINE, ROUTINE_POSITION_COUNT)
{
}

void ZRoutineController::start(unsigned long startTime)
{
  // SerialUSB.println("Starting routine.");
  routine.reset();
  planNextCommand(startTime);
}

void ZRoutineController::loop(unsigned long currentTime)
{
  if (routine.isCompleted())
    return;

  while (unsigned long completionTime = currentCommandCompleted(currentTime)) {
    if (currentCommandPath) {
      //we need to capture the last point on this path segment to use as the starting
      //position to plan the move along the next path segment
      pushCurrentPathPosition(1.0d);
      delete currentCommandPath;
      currentCommandPath = NULL;
    }

    if (!planNextCommand(completionTime))
      return;
  }

  if (currentCommandPath) {
    double interpolatedTime = ((double)(currentTime - currentCommandStartTime)) / ((double)currentCommand->timing);
    pushCurrentPathPosition(interpolatedTime);
  }
}

void ZRoutineController::pushCurrentPathPosition(double interpolatedTime)
{
  //we need to capture the last point on this path segment to use as the starting
  //position to plan the move along the next path segment
  currentCommandPath->interpolate(interpolatedTime, &currentTargetPosition);
  if (currentCommandPath->updatesPosition())
    zippy->setTargetPosition(&currentTargetPosition);
  else
    zippy->setTargetOrientation(&currentTargetPosition.orientation);
}

bool ZRoutineController::planNextCommand(unsigned long commandStartTime)
{
  //plan the next path command
  currentCommand = routine.getNextCommand();
  if (!currentCommand)
    return false;

  //begin execution of the new command
  currentCommandStartTime = commandStartTime;
  
  //plan the path for this command
  if (currentCommandPath)
    delete currentCommandPath;
  currentCommandPath = convertToPath(&currentTargetPosition, currentCommand);

  return true;
}

unsigned long ZRoutineController::currentCommandCompleted(unsigned long currentTime)
{
  unsigned long commandEndTime = currentCommandStartTime + currentCommand->timing;
  if (currentTime >= commandEndTime)
    return commandEndTime;

  return 0;
}

void ZRoutineController::stop()
{
  if (currentCommandPath) {
    delete currentCommandPath;
    currentCommandPath = NULL;
  }
}

const ZPath* convertToPath(const KMatrix2* start, const Command* command)
{
  switch (command->type) {
    case CommandMoveTo:
      return planPath(start, command->params.p1+ZIPPY_OFFSET_X, command->params.p2, command->params.p3);
    case CommandTurnTo:
      return new Turn(start->orientation.get(), subtractAngles(command->params.p1, start->orientation.get()));
    case CommandMove:
      return new Move(start, command->params.p1);
    case CommandArc:
      return new Arc(start, command->params.p1, command->params.p2);
    case CommandTurn:
      return new Turn(start->orientation.get(), command->params.p1);
  }

  return NULL;
}
