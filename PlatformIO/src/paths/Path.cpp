
#include "zippies/config/BodyConfig.h"
#include "zippies/paths/Path.h"
#include "zippies/paths/Turn.h"
#include "zippies/paths/Move.h"
#include "zippies/paths/Arc.h"

void Path::setPathSegments(
    const KMatrix2* ap,
    PathDefinition* ps,
    int psc)
{
  pathAnchorPosition.set(ap);
  pathSegments = ps;
  pathSegmentCount = psc;
  pathLength = 0.0d;

  reset();

  if (!pathSegmentCount) {
    endCurrentSegment();
    return;
  }

  //calculate the total path length
  for (int i = 0; i < pathSegmentCount; i++)
    pathLength += getPathSegmentLength(&pathSegments[i]);
  planCurrentPathSegment();
}

void Path::reset()
{
  currentPathSegmentIndex = 0;
  currentPathSegmentStartPosition = 0.0d;
  currentPathSegmentAnchorPosition.set(&pathAnchorPosition);
  currentPathSegmentLength = 0.0d;
}

void Path::endCurrentSegment()
{
  if (!currentPathSegment)
    return;

  delete currentPathSegment;
  currentPathSegment = NULL;
  currentPathSegmentLength = 0.0d;
}

void Path::planCurrentPathSegment()
{
  // SerialUSB.print("Planning path segment: ");
  // SerialUSB.println(currentPathSegmentIndex);

  if (currentPathSegment)
    delete currentPathSegment;

  PathDefinition* nextPathSegment = &pathSegments[currentPathSegmentIndex];
  switch (nextPathSegment->type) {

    case PathDefinitionType::Move:
      currentPathSegment = new Move(
          &currentPathSegmentAnchorPosition,
          nextPathSegment->params.p1);
      currentPathSegmentLength = abs(nextPathSegment->params.p1);
      break;

    case PathDefinitionType::Arc:
      currentPathSegment = new Arc(
          &currentPathSegmentAnchorPosition,
          nextPathSegment->params.p1,
          nextPathSegment->params.p2);
      currentPathSegmentLength = abs(nextPathSegment->params.p1 * nextPathSegment->params.p2);
      break;

    case PathDefinitionType::Turn:
      currentPathSegment = new Turn(
          currentPathSegmentAnchorPosition.orientation.get(),
          nextPathSegment->params.p1);
      currentPathSegmentLength = WHEEL_CENTER_BODY_CENTER_OFFSET_X * abs(nextPathSegment->params.p1);
      break;

    default:
      currentPathSegment = NULL;
      currentPathSegmentLength = 0.0d;
      break;
  }
}

bool Path::interpolate(
  double interpolatedTime,
  KMatrix2* targetPosition)
{
  if (!pathSegmentCount)
    return false;

  /*
  SerialUSB.print("Interpolating path segment: ");
  SerialUSB.print(currentPathSegmentIndex);
  SerialUSB.print(" - ");
  SerialUSB.println(interpolatedTime);
  */

  double positionAlongPath = pathLength * interpolatedTime;
  if (positionAlongPath < currentPathSegmentStartPosition) {
    reset();
    planCurrentPathSegment();
  }

  while (currentPathSegmentIndex < pathSegmentCount && positionAlongPath > currentPathSegmentStartPosition + currentPathSegmentLength) {
    currentPathSegment->interpolate(1.0d, &currentPathSegmentAnchorPosition);
    currentPathSegmentStartPosition += currentPathSegmentLength;
    currentPathSegmentIndex++;
    planCurrentPathSegment();
  }

  double interpolatedPathSegmentPosition = min((positionAlongPath - currentPathSegmentStartPosition) / currentPathSegmentLength, 1.0d);
  currentPathSegment->interpolate(interpolatedPathSegmentPosition, targetPosition);

  /*
  SerialUSB.print("Path segment interpolation complete: ");
  SerialUSB.println(currentPathSegmentIndex);
  */
  return pathSegments[currentPathSegmentIndex].type != PathDefinitionType::Turn;
}

double getPathSegmentLength(const PathDefinition* pathSegment)
{
  switch (pathSegment->type) {
    case PathDefinitionType::Move:
      return abs(pathSegment->params.p1);
    case PathDefinitionType::Turn:
      return WHEEL_CENTER_BODY_CENTER_OFFSET_X * abs(pathSegment->params.p1);
    case PathDefinitionType::Arc:
      return abs(pathSegment->params.p1 * pathSegment->params.p2);
  }

  return 0.0d;
}

double getPathLength(const PathDefinition* path, int pathSegmentCount)
{
  double totalLength = 0.0d;
  for (int i = 0; i < pathSegmentCount; i++)
    totalLength += getPathSegmentLength(&path[i]);
  return totalLength;
}
