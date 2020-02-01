
#ifndef _PATH_H_
#define _PATH_H_

#include "zippies/ZippyMath.h"
#include "PathSegment.h"

typedef enum class _MovementState
{
  Stopped,
  Turning,
  Moving,
} MovementState;

typedef enum class _PathDefinitionType
{
  //all commands below this point are relative to the current position and orientation
  Move,         //straight move forward or backward
  Turn,         //arc with no radius
  Arc,          //arc with radius and subtended angle
} PathDefinitionType;

typedef struct _PathDefinition
{
  PathDefinitionType type;
  struct {
    double p1, p2;
  } params;
} PathDefinition;

double getPathSegmentLength(const PathDefinition* pathSegment);

class Path
{

private:
  PathDefinition* pathSegments;
  int pathSegmentCount = 0;
  int currentPathSegmentIndex = 0;
  double currentPathSegmentStartPosition = 0.0d;
  double pathLength = 0.0d;

  KMatrix2 currentPathSegmentAnchorPosition;
  PathSegment* currentPathSegment = NULL;
  double currentPathSegmentLength = 0.0d;
  MovementState currentPathSegmentMovementState = MovementState::Stopped;

  void planCurrentPathSegment();
  void endCurrentSegment();

public:
  Path() {}

  void setPathSegments(const KMatrix2* anchorPosition, PathDefinition* ps, int psc);
  double getLength() const { return pathLength; }
  MovementState getMovementState() const { return currentPathSegmentMovementState; }

  void interpolate(
    double interpolatedTime,
    KMatrix2* targetPosition);

  ~Path() {
    if (currentPathSegment)
      delete currentPathSegment;
  }

};

#endif
