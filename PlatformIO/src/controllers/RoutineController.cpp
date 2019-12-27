
#include "zippies/ZippyControllers.h"
#include "zippies/ZippyMath.h"

#include "zippies/config/BodyConfig.h"
#include "paths/Turn.h"
#include "paths/Move.h"
#include "paths/Arc.h"
#include "paths/ZPathPlanner.h"

#define EASING_RATIO                             0.25d

double bezierEaseInOut(double t, double a, double b, double c, double d);
double bezierEaseInOut(double t, double a, double b);
double quadEaseInOut(double t, double b, double c, double d);
double cubicEaseInOut(double t, double b, double c, double d);
double easeInOut(double t, double c1, double c2);

RoutineController::RoutineController(Zippy* z)
  : zippy(z)
{
}

void RoutineController::setRoutine(PathSegment* c, int cc)
{
  commands = c;
  commandCount = cc;
}

void RoutineController::start(unsigned long startTime)
{
  currentCommandIndex = 0;
  planCurrentCommand(startTime);
}

void RoutineController::planCurrentCommand(unsigned long currentTime)
{
  currentCommandLength = 0.0d;
  for (int i = 0; i < commands[currentCommandIndex].movementCount; i++)
    currentCommandLength += getMovementLength(&commands[currentCommandIndex].movements[i]);
  currentCommandLength *= commands[currentCommandIndex].movementRepeatCount + 1;
  currentCommandPosition = 0.0d;
  currentCommandStartTime = currentTime;

  currentMovementIndex = 0;
  currentMovementLoopCount = 0;
  plotCurrentMovement();
}

void RoutineController::plotCurrentMovement()
{
  if (currentMovement)
    delete currentMovement;

  Movement* nextMovement = &commands[currentCommandIndex].movements[currentMovementIndex];
  switch (nextMovement->type) {
    case MovementType::Move:
      currentMovement = new Move(
          &currentTargetPosition,
          nextMovement->params.p1);
      currentMovementLength = abs(nextMovement->params.p1);
      break;
    case MovementType::Turn:
      currentMovement = new Turn(
          currentTargetPosition.orientation.get(),
          nextMovement->params.p1);
      currentMovementLength = WHEEL_CENTER_BODY_CENTER_OFFSET_X * abs(nextMovement->params.p1);
      break;
    case MovementType::Arc:
      currentMovement = new Arc(
          &currentTargetPosition,
          nextMovement->params.p1,
          nextMovement->params.p2);
      currentMovementLength = abs(nextMovement->params.p1 * nextMovement->params.p2);
      break;
    default:
      currentMovement = NULL;
      currentMovementLength = 0.0d;
      break;
  }
}

double RoutineController::getMovementLength(const Movement* movement)
{
  switch (movement->type) {
    case MovementType::Move:
      return abs(movement->params.p1);
    case MovementType::Turn:
      return WHEEL_CENTER_BODY_CENTER_OFFSET_X * abs(movement->params.p1);
    case MovementType::Arc:
      return abs(movement->params.p1 * movement->params.p2);
  }

  return 0.0d;
}

void RoutineController::loop(unsigned long currentTime)
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

    currentCommandIndex++;
    if (currentCommandIndex >= commandCount)
      return;

    planCurrentCommand(completionTime);
  }

  double commandInterpolatedTime = ((double)(currentTime - currentCommandStartTime)) / ((double)commands[currentCommandIndex].timing);
  // /*
  commandInterpolatedTime = bezierEaseInOut(
      commandInterpolatedTime,
      commands[currentCommandIndex].easeInFactor,
      1.0d - commands[currentCommandIndex].easeOutFactor);
  // */
  double movementPosition = (currentCommandLength * commandInterpolatedTime) - currentCommandPosition;
  while (movementPosition >= currentMovementLength) {
    if (currentMovementIndex >= commands[currentCommandIndex].movementCount-1) {
      if (currentMovementLoopCount >= commands[currentCommandIndex].movementRepeatCount)
        return;

      //start the next loop
      currentMovementLoopCount++;
      currentMovementIndex = 0;
    }
    else
      currentMovementIndex++;

    //we need to capture the last point on this path segment to use as the starting
    //position to plan the move along the next path segment
    currentMovement->interpolate(1.0d, &currentTargetPosition);

    //adjust our movement positions
    currentCommandPosition += currentMovementLength;
    movementPosition -= currentMovementLength;

    //plot the next movement
    plotCurrentMovement();
  }

  //push an updated position
  double movementInterpolatedTime = movementPosition / currentMovementLength;
  pushCurrentPathPosition(movementInterpolatedTime);
}

double bezierEaseInOut(double t, double a, double b, double c, double d)
{
  double u = 1.0d - t;
  double u2 = u * u;
  double u3 = u2 * u;
  double t2 = t * t;
  double t3 = t2 * t;

  double u2t3 = 3.0d * u2 * t;
  double ut23 = 3.0d * u * t2;
  return (u3 * a) + (u2t3 * b) + (ut23 * c) + (t3 * d);
}

double bezierEaseInOut(double t, double a, double b)
{
  double u = 1.0d - t;
  double u2 = u * u;
  double t2 = t * t;
  double t3 = t2 * t;

  double u2t3 = 3.0d * u2 * t;
  double ut23 = 3.0d * u * t2;
  return (u2t3 * a) + (ut23 * b) + t3;
}

void RoutineController::pushCurrentPathPosition(double interpolatedTime)
{
  currentMovement->interpolate(interpolatedTime, &currentTargetPosition);
  if (currentMovement->updatesPosition())
    zippy->setTargetPosition(&currentTargetPosition);
  else
    zippy->setTargetOrientation(&currentTargetPosition.orientation);
}

unsigned long RoutineController::currentCommandCompleted(unsigned long currentTime)
{
  unsigned long commandEndTime = currentCommandStartTime + commands[currentCommandIndex].timing;
  if (currentTime >= commandEndTime)
    return commandEndTime;

  return 0;
}

void RoutineController::stop()
{
  if (currentMovement) {
    delete currentMovement;
    currentMovement = NULL;
  }
}

double quadEaseInOut(double t, double b, double c, double d)
{
  t /= d/2.0d;
  if (t < 1.0d)
      return c/2.0d*t*t + b;

  t--;
  return -c/2.0d * (((t-2.0d)*t) - 1.0d) + b;
}

double cubicEaseInOut(double t, double b, double c, double d)
{
  t /= d/2.0d;
  if (t < 1.0d)
      return c/2.0d*t*t*t + b;

  t -= 2.0d;
  return c/2.0d * (t*t*t + 2.0d) + b;
}
