
#include "zippies/paths/Routine.h"

#include "zippies/config/BodyConfig.h"
#include "zippies/paths/Turn.h"
#include "zippies/paths/Move.h"
#include "zippies/paths/Arc.h"

double bezierEaseInOut(double t, double a, double b);
/*
double bezierEaseInOut(double t, double a, double b, double c, double d);
double quadEaseInOut(double t, double b, double c, double d);
double cubicEaseInOut(double t, double b, double c, double d);
double easeInOut(double t, double c1, double c2);
*/

void Routine::setRoutineSegments(const ZMatrix2* ap, RoutineDefinition* rs, int rsc)
{
  currentTargetPosition.set(ap);
  routineSegments = rs;
  routineSegmentCount = rsc;

  currentRoutineSegmentIndex = 0;
}

void Routine::start(unsigned long startTime)
{
  currentRoutineSegmentIndex = 0;
  planCurrentRoutineSegment(startTime);
}

void Routine::planCurrentRoutineSegment(unsigned long currentTime)
{
  // SerialUSB.print("Planning routine segment: ");
  // SerialUSB.println(currentRoutineSegmentIndex);
  path.setPathSegments(
      &currentTargetPosition,
      routineSegments[currentRoutineSegmentIndex].pathSegments,
      routineSegments[currentRoutineSegmentIndex].pathSegmentCount);
  currentRoutineSegmentStartTime = currentTime;

  if (!routineSegments[currentRoutineSegmentIndex].pathSegmentCount)
    currentMovementState = MovementState::Stopped;
}

void Routine::loop(unsigned long currentTime)
{
  if (isRoutineCompleted())
    return;

  //process the next routine segment
  while (unsigned long completionTime = currentRoutineSegmentCompleted(currentTime)) {
    // SerialUSB.print("Completed routine segment: ");
    // SerialUSB.println(currentRoutineSegmentIndex);
    if (routineSegments[currentRoutineSegmentIndex].pathSegmentCount) {
      if (path.interpolate(1.0, &currentTargetPosition))
        currentMovementState = MovementState::Moving;
      else
        currentMovementState = MovementState::Turning;
    }

    currentRoutineSegmentIndex++;
    if (currentRoutineSegmentIndex >= routineSegmentCount) {
      // SerialUSB.println("Routine completed.");
      currentMovementState = MovementState::Stopped;
      return;
    }

    planCurrentRoutineSegment(completionTime);
  }

  if (routineSegments[currentRoutineSegmentIndex].pathSegmentCount) {
    unsigned long repeatedTime = ((currentTime - currentRoutineSegmentStartTime) * (routineSegments[currentRoutineSegmentIndex].pathRepeatCount+1)) %
        routineSegments[currentRoutineSegmentIndex].timing;
    double currentRoutineSegmentInterpolatedTime =
        ((double)repeatedTime) /
        ((double)routineSegments[currentRoutineSegmentIndex].timing);
    // /*
    currentRoutineSegmentInterpolatedTime = bezierEaseInOut(
        currentRoutineSegmentInterpolatedTime,
        routineSegments[currentRoutineSegmentIndex].easeInFactor,
        1.0d - routineSegments[currentRoutineSegmentIndex].easeOutFactor);
    // */

    if (path.interpolate(currentRoutineSegmentInterpolatedTime, &currentTargetPosition))
      currentMovementState = MovementState::Moving;
    else
      currentMovementState = MovementState::Turning;
  }
}

unsigned long Routine::currentRoutineSegmentCompleted(unsigned long currentTime)
{
  unsigned long currentRoutineSegmentEndTime = currentRoutineSegmentStartTime + routineSegments[currentRoutineSegmentIndex].timing;
  if (currentTime >= currentRoutineSegmentEndTime)
    return currentRoutineSegmentEndTime;

  return 0;
}

double getRoutineLength(const RoutineDefinition* routine, int routineSegmentCount)
{
  double totalLength = 0.0;
  for (int i = 0; i < routineSegmentCount; i++)
    totalLength += getPathLength(routine[i].pathSegments, routine[i].pathSegmentCount);
  return totalLength;
}

double bezierEaseInOut(double t, double a, double b, double c, double d)
{
  double u = 1.0 - t;
  double u2 = u * u;
  double u3 = u2 * u;
  double t2 = t * t;
  double t3 = t2 * t;

  double u2t3 = 3.0 * u2 * t;
  double ut23 = 3.0 * u * t2;
  return (u3 * a) + (u2t3 * b) + (ut23 * c) + (t3 * d);
}

double bezierEaseInOut(double t, double a, double b)
{
  double t2 = t * t;
  double t3 = t2 * t;
  double mt = 1.0 - t;
  double mt2 = mt * mt;
  return (a * 3.0 * mt2 * t) + (b * 3.0 * mt * t2) + t3;
}

double quadBezierEaseInOut2(double t, double a)
{
  double t2 = t * t;
  double mt = 1.0 - t;
  return (a * 2.0 * mt * t) + t2;
}

double quadEaseInOut(double t, double b, double c, double d)
{
  t /= d/2.0;
  if (t < 1.0)
      return c/2.0*t*t + b;

  t--;
  return -c/2.0 * (((t-2.0)*t) - 1.0) + b;
}

double cubicEaseInOut(double t, double b, double c, double d)
{
  t /= d/2.0;
  if (t < 1.0)
      return c/2.0*t*t*t + b;

  t -= 2.0;
  return c/2.0 * (t*t*t + 2.0) + b;
}

void reversePath(
  PathDefinition* pathSegments,
  int pathSegmentCount)
{
  for (int i = 0; i < pathSegmentCount; i++)
    pathSegments[i].params.p1 = -pathSegments[i].params.p1;
}

void reverseRoutine(
  RoutineDefinition* routine,
  int routineCount)
{
  for (int i = 0; i < routineCount; i++)
    reversePath(routine[i].pathSegments, routine[i].pathSegmentCount);
}
