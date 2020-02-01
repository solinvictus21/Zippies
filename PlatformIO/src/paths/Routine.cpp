
#include "zippies/paths/Routine.h"

#include "zippies/config/BodyConfig.h"
#include "zippies/paths/Turn.h"
#include "zippies/paths/Move.h"
#include "zippies/paths/Arc.h"
// #include "paths/ZPathPlanner.h"

#define EASING_RATIO                             0.25d

double bezierEaseInOut(double t, double a, double b, double c, double d);
double bezierEaseInOut(double t, double a, double b);
double quadEaseInOut(double t, double b, double c, double d);
double cubicEaseInOut(double t, double b, double c, double d);
double easeInOut(double t, double c1, double c2);

void Routine::setRoutineSegments(const KMatrix2* ap, RoutineDefinition* rs, int rsc)
{
  currentTargetPosition.set(ap);
  routineSegments = rs;
  routineSegmentCount = rsc;
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
}

void Routine::loop(unsigned long currentTime)
{
  if (isRoutineCompleted())
    return;

  //process the next routine segment
  while (unsigned long completionTime = currentRoutineSegmentCompleted(currentTime)) {
    // SerialUSB.print("Completed routine segment: ");
    // SerialUSB.println(currentRoutineSegmentIndex);
    path.interpolate(1.0d, &currentTargetPosition);
    currentRoutineSegmentIndex++;
    if (currentRoutineSegmentIndex >= routineSegmentCount) {
      // SerialUSB.println("Routine completed.");
      return;
    }

    planCurrentRoutineSegment(completionTime);
  }

  double currentRoutineSegmentInterpolatedTime =
      ((double)(currentTime - currentRoutineSegmentStartTime)) /
      ((double)routineSegments[currentRoutineSegmentIndex].timing);
  currentRoutineSegmentInterpolatedTime = bezierEaseInOut(
      currentRoutineSegmentInterpolatedTime,
      routineSegments[currentRoutineSegmentIndex].easeInFactor,
      1.0d - routineSegments[currentRoutineSegmentIndex].easeOutFactor);
  path.interpolate(currentRoutineSegmentInterpolatedTime, &currentTargetPosition);
}

/*
void Routine::loopDebug(unsigned long currentTime, ZippyFace* face)
{
  // face->displayData(0, 30, currentTime);
  if (isRoutineCompleted())
    return;

  //process the next routine segment
  face->displayData(0, 10, currentTime);
  while (unsigned long completionTime = currentRoutineSegmentCompleted(currentTime)) {
    // SerialUSB.print("Completed routine segment: ");
    // SerialUSB.println(currentRoutineSegmentIndex);
    // face->displayData(0, 30, currentTime);
    path.interpolate(1.0d, &currentTargetPosition);
    currentRoutineSegmentIndex++;
    if (currentRoutineSegmentIndex >= routineSegmentCount) {
      // SerialUSB.println("Routine completed.");
      return;
    }

    face->displayData(0, 20, currentTime);
    face->displayData(SCREEN_WIDTH_PIXELS_2, 20, currentRoutineSegmentIndex);
    face->displayData(SCREEN_WIDTH_PIXELS_2, 30, routineSegmentCount);
    planCurrentRoutineSegment(completionTime);
    face->displayData(0, 30, currentTime);
  }

  // face->displayData(0, 30, currentTime);
  double currentRoutineSegmentInterpolatedTime =
      ((double)(currentTime - currentRoutineSegmentStartTime)) /
      ((double)routineSegments[currentRoutineSegmentIndex].timing);
  path.interpolate(currentRoutineSegmentInterpolatedTime, &currentTargetPosition);
}
*/

unsigned long Routine::currentRoutineSegmentCompleted(unsigned long currentTime)
{
  unsigned long currentRoutineSegmentEndTime = currentRoutineSegmentStartTime + routineSegments[currentRoutineSegmentIndex].timing;
  if (currentTime >= currentRoutineSegmentEndTime)
    return currentRoutineSegmentEndTime;

  return 0;
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
