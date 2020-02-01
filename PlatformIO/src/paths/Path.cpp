
#include "zippies/config/BodyConfig.h"
#include "zippies/paths/Path.h"
#include "zippies/paths/Turn.h"
#include "zippies/paths/Move.h"
#include "zippies/paths/Arc.h"

void Path::setPathSegments(const KMatrix2* ap, PathDefinition* ps, int psc)
{
  pathSegments = ps;
  pathSegmentCount = psc;
  currentPathSegmentIndex = 0;
  currentPathSegmentStartPosition = 0.0d;
  pathLength = 0.0d;
  currentPathSegmentAnchorPosition.set(ap);

  if (pathSegmentCount == 0) {
    endCurrentSegment();
    return;
  }

  //calculate the total path length
  for (int i = 0; i < pathSegmentCount; i++)
    pathLength += getPathSegmentLength(&pathSegments[i]);

  planCurrentPathSegment();
}

void Path::endCurrentSegment()
{
  if (!currentPathSegment)
    return;

  delete currentPathSegment;
  currentPathSegment = NULL;
  currentPathSegmentLength = 0.0d;
  currentPathSegmentMovementState = MovementState::Stopped;
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
      currentPathSegmentMovementState = MovementState::Moving;
      break;

    case PathDefinitionType::Arc:
      currentPathSegment = new Arc(
          &currentPathSegmentAnchorPosition,
          nextPathSegment->params.p1,
          nextPathSegment->params.p2);
      currentPathSegmentLength = abs(nextPathSegment->params.p1 * nextPathSegment->params.p2);
      currentPathSegmentMovementState = MovementState::Moving;
      break;

    case PathDefinitionType::Turn:
      currentPathSegment = new Turn(
          currentPathSegmentAnchorPosition.orientation.get(),
          nextPathSegment->params.p1);
      currentPathSegmentLength = WHEEL_CENTER_BODY_CENTER_OFFSET_X * abs(nextPathSegment->params.p1);
      currentPathSegmentMovementState = MovementState::Turning;
      break;

    default:
      currentPathSegment = NULL;
      currentPathSegmentLength = 0.0d;
      currentPathSegmentMovementState = MovementState::Stopped;
      break;

  }
}

void Path::interpolate(
  double interpolatedTime,
  KMatrix2* targetPosition)
{
  if (currentPathSegmentIndex >= pathSegmentCount)
    return;

  double positionAlongPath = pathLength * interpolatedTime;
  while (positionAlongPath - currentPathSegmentStartPosition > currentPathSegmentLength) {
    // SerialUSB.print("Completed path segment: ");
    // SerialUSB.println(currentPathSegmentIndex);
    currentPathSegment->interpolate(1.0d, &currentPathSegmentAnchorPosition);
    currentPathSegmentIndex++;
    if (currentPathSegmentIndex >= pathSegmentCount) {
      // SerialUSB.println("Path completed.");
      targetPosition->set(&currentPathSegmentAnchorPosition);
      endCurrentSegment();
      return;
    }

    currentPathSegmentStartPosition += currentPathSegmentLength;
    planCurrentPathSegment();
  }

  if (currentPathSegment) {
    double interpolatedPathSegmentPosition = constrain(
        (positionAlongPath - currentPathSegmentStartPosition) / currentPathSegmentLength,
        0.0d, 1.0d);
    currentPathSegment->interpolate(interpolatedPathSegmentPosition, targetPosition);
  }
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
