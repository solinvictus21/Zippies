
#include "FollowPath.h"
#include "LinePath.h"
#include "BezierPath.h"

FollowPath::FollowPath(KVector2* cp, int cpc, unsigned long executionTime)
  : ZippyCommand(executionTime),
    controlPoints(cp),
    controlPointCount(cpc)
{
  //in a bezier path, the first and last points are not control points, but are instead endpoints
  anchorPoints = new KVector2*[controlPointCount-1];
  for (int i = 0; i < controlPointCount-1; i++) {
    anchorPoints[i] = new KVector2((controlPoints[i].getX() + controlPoints[i+1].getX()) / 2.0d,
        (controlPoints[i].getY() + controlPoints[i+1].getY()) / 2.0d);
  }
  pathSegments = new KPath*[controlPointCount];
  pathSegments[0] = new LinePath(&controlPoints[0], anchorPoints[0]);
  for (int i = 1; i < controlPointCount-1; i++) {
    pathSegments[i] = new BezierPath(anchorPoints[i-1], &controlPoints[i], anchorPoints[i]);
  }
  pathSegments[controlPointCount-1] = new LinePath(anchorPoints[controlPointCount-2], &controlPoints[controlPointCount-1]);

  //calculate the total length
  totalLength = 0;
  for (int i = 0; i < controlPointCount; i++)
    totalLength += pathSegments[i]->getLength();
}

bool FollowPath::getPosition(unsigned long atDeltaTime, KVector2* targetPosition)
{
  //from the time, calculate the distance we should be through the entire path
  if (atDeltaTime > previousTime) {
    currentDistanceAlongSegment += totalLength * (((double)(atDeltaTime - previousTime)) / ((double)executionTime));
  }
  else {
    //start over
    currentPathSegment = 0;
    currentDistanceAlongSegment = totalLength * (((double)atDeltaTime) / ((double)executionTime));
  }
  previousTime = atDeltaTime;

  while (currentDistanceAlongSegment > pathSegments[currentPathSegment]->getLength()) {
    //the look-ahead distance is beyond the end of the current path segment
    if (currentPathSegment+1 == controlPointCount) {
      //but this is the last segment, so stop at the end
      currentDistanceAlongSegment = pathSegments[currentPathSegment]->getLength();
      break;
    }

    //there are more path segments, so move to the next path segment
    //first determine the distance left over after completing the current segment
    currentDistanceAlongSegment -= pathSegments[currentPathSegment]->getLength();

    //move to the next path segment
    currentPathSegment++;
  }

  //start by determining the current distance we've traveled along this segment, normalized from 0.0 to 1.0
  double deltaAlongPath = currentDistanceAlongSegment / pathSegments[currentPathSegment]->getLength();

  //now LERP out the x/y position along the current path segment
  pathSegments[currentPathSegment]->lerp(deltaAlongPath, targetPosition);
  // targetPosition->printDebug();

  return true;
}

FollowPath::~FollowPath()
{
  for (int i = 0; i < controlPointCount-1; i++)
    delete pathSegments[i];
  delete[] pathSegments;

  for (int i = 0; i < controlPointCount-2; i++)
    delete anchorPoints[i];
  delete[] anchorPoints;
}
