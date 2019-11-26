
#include "ZippyConfig.h"
#include "ZRoutineController.h"
#include "paths/Turn.h"
#include "paths/Move.h"
#include "paths/Arc.h"
#include "paths/CompositePath.h"
#include "paths/ZPathPlanner.h"
#include "lighthouse/KVector3.h"

#define EASING_RATIO                             0.25d

double quadEaseInOut(double t, double b, double c, double d);
double easeInOut(double t, double c1, double c2);
// const ZPath* convertToPath(const KMatrix2* start, const Command* command);

ZRoutineController::ZRoutineController(Zippy* z)
  : zippy(z)/*,
    routine(ROUTINE, ROUTINE_POSITION_COUNT)*/
{
}

void ZRoutineController::setRoutine(Command2* c, int cc)
{
  commands = c;
  commandCount = cc;
}

void ZRoutineController::start(unsigned long startTime)
{
  /*
  routine.reset();
  planNextCommand(startTime);
  */

  currentCommand = 0;
  planCurrentCommand(startTime);
}

void ZRoutineController::planCurrentCommand(unsigned long currentTime)
{
  currentCommandLength = 0.0d;
  for (int i = 0; i < commands[currentCommand].movementCount; i++)
    currentCommandLength += getMovementLength(&commands[currentCommand].movements[i]);
  currentCommandLength *= commands[currentCommand].movementRepeatCount + 1;
  currentCommandPosition = 0.0d;
  currentCommandStartTime = currentTime;

  currentMovementIndex = 0;
  currentMovementLoopCount = 0;
  plotCurrentMovement();
}

void ZRoutineController::plotCurrentMovement()
{
  if (currentMovement) {
    currentCommandPosition += currentMovement->getLength();
    delete currentMovement;
    currentMovement = NULL;
  }

  switch (commands[currentCommand].movements[currentMovementIndex].type) {
    case MovementType::Move:
      currentMovement = new Move(
          &currentTargetPosition,
          commands[currentCommand].movements[currentMovementIndex].params.p1);
      break;
    case MovementType::Turn:
      currentMovement = new Turn(
          currentTargetPosition.orientation.get(),
          commands[currentCommand].movements[currentMovementIndex].params.p1);
      break;
    case MovementType::Arc:
      currentMovement = new Arc(
          &currentTargetPosition,
          commands[currentCommand].movements[currentMovementIndex].params.p1,
          commands[currentCommand].movements[currentMovementIndex].params.p2);
      break;
  }
}

double ZRoutineController::getMovementLength(const Movement* movement)
{
  switch (movement->type) {
    case MovementType::Move:
      return abs(movement->params.p1);
    case MovementType::Turn:
      return WHEEL_OFFSET_X * abs(movement->params.p1);
    case MovementType::Arc:
      return (abs(movement->params.p1) + WHEEL_OFFSET_X) * abs(movement->params.p2);
  }

  return 0.0d;
}

/*
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
// */

void ZRoutineController::loop(unsigned long currentTime)
{
  // if (routine.isCompleted())
  if (isRoutineCompleted())
    return;

  //process the next command
  while (unsigned long completionTime = currentCommandCompleted(currentTime)) {
    if (currentMovement) {
      //we need to capture the last point on this path segment to use as the starting
      //position to plan the move along the next path segment
      pushCurrentPathPosition(1.0d);
      delete currentMovement;
      currentMovement = NULL;
    }

    currentCommand++;
    if (currentCommand >= commandCount)
      return;

    planCurrentCommand(completionTime);
  }

  // double pathInterpolatedTime = ((double)(currentTime - currentCommandStartTime)) / ((double)commands[currentCommand].timing);
  double pathInterpolatedTime = quadEaseInOut(((double)(currentTime - currentCommandStartTime)), 0.0d, 1.0d, ((double)commands[currentCommand].timing) * 0.5d);
  double movementPosition = (currentCommandLength * pathInterpolatedTime) - currentCommandPosition;
  while (movementPosition >= currentMovement->getLength()) {
    movementPosition -= currentMovement->getLength();
    pushCurrentPathPosition(1.0d);

    currentMovementIndex++;
    if (currentMovementIndex >= commands[currentCommand].movementCount) {
      currentMovementLoopCount++;
      if (currentMovementLoopCount > commands[currentCommand].movementRepeatCount)
        return;
      currentMovementIndex = 0;
    }

    plotCurrentMovement();
  }
  double movementInterpolatedTime = movementPosition / currentMovement->getLength();
  pushCurrentPathPosition(movementInterpolatedTime);
}

void ZRoutineController::pushCurrentPathPosition(double interpolatedTime)
{
  //we need to capture the last point on this path segment to use as the starting
  //position to plan the move along the next path segment
  // currentCommandPath->interpolate(interpolatedTime, &currentTargetPosition);
  currentMovement->interpolate(interpolatedTime, &currentTargetPosition);
  // currentMovement->interpolate(easeInOut(interpolatedTime, EASING_RATIO, 1.0d-EASING_RATIO), &currentTargetPosition);
  if (currentMovement->updatesPosition())
    zippy->setTargetPosition(&currentTargetPosition);
  else
    zippy->setTargetOrientation(&currentTargetPosition.orientation);
}

unsigned long ZRoutineController::currentCommandCompleted(unsigned long currentTime)
{
  // unsigned long commandEndTime = currentCommandStartTime + currentCommand->timing;
  // unsigned long commandEndTime = currentPathStartTime + currentCommand->timing;
  unsigned long commandEndTime = currentCommandStartTime + commands[currentCommand].timing;
  if (currentTime >= commandEndTime)
    return commandEndTime;

  return 0;
}

void ZRoutineController::stop()
{
  /*
  if (currentCommandPath) {
    delete currentCommandPath;
    currentCommandPath = NULL;
  }
  */
  if (currentMovement) {
    delete currentMovement;
    currentMovement = NULL;
  }
}

float linearEase(double t, double b, double c, double d) {
  t /= d;
  return b + (c*t);
}

float quadEaseIn(double t, double b, double c, double d) {
  t /= d;
  return b + (c*t*t*t);
}

float quadEaseOut(double t, double b, double c, double d) {
  t /= d;
  return b - (c*t*(t-2.0d));
}

double quadEaseInOut(double t, double b, double c, double d)
{
  t /= d;
  if (t < 1.0d)
      return c/2.0d*t*t + b;

  t--;
  return -c/2.0d * (t*(t-2.0d) - 1.0d) + b;
}

double easeInOut(double t, double c1, double c2)
{
  return quadEaseInOut(t, 0.0d, 1.0d, 0.25d);
  /*
  if (t < c1)
    // return linearEase(t, 0.0d, c1, c1);
    return quadEaseIn(t, 0.0d, c1, c1);

  double range = c2 - c1;
  if (t < c2)
    return linearEase(t - c1, c1, range, range);

  range = 1.0d - c2;
  // return linearEase(t - c2, c2, range, range);
  return quadEaseOut(t - c2, c2, range, range);
  */
}

/*
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
*/
