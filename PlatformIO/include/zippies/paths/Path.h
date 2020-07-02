
#ifndef _PATH_H_
#define _PATH_H_

#include "zippies/ZippyMath.h"
#include "PathSegment.h"

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
double getPathLength(const PathDefinition* path, int pathSegmentCount);

class Path
{

private:
  ZMatrix2 pathAnchorPosition;
  PathDefinition* pathSegments;
  int pathSegmentCount = 0;
  double pathLength = 0.0d;

  int currentPathSegmentIndex = 0;
  double currentPathSegmentStartPosition = 0.0d;

  ZMatrix2 currentPathSegmentAnchorPosition;
  PathSegment* currentPathSegment = NULL;
  double currentPathSegmentLength = 0.0d;

  void reset();
  void planCurrentPathSegment();
  void endCurrentSegment();

public:
  Path() {}

  void setPathSegments(
      const ZMatrix2* anchorPosition,
      PathDefinition* pathDefinition,
      int pathDefinitionCount);
  double getLength() const { return pathLength; }

  bool interpolate(
    double interpolatedTime,
    ZMatrix2* targetPosition);

  ~Path() {
    if (currentPathSegment)
      delete currentPathSegment;
  }

};

#endif
