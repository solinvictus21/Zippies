
#ifndef _FOLLOWPATH_H_
#define _FOLLOWPATH_H_

#include <PID_v1.h>
#include "ZippyCommand.h"
#include "../lighthouse/KBezier2.h"
#include "../Zippy.h"
#include "../lighthouse/KVector2.h"

class FollowPath : public ZippyCommand
{

private:
  KVector2* controlPoints;
  int controlPointCount;

  //calculated and dynamically allocated based on the set of control points
  KVector2** anchorPoints;
  KPath** pathSegments;
  double totalLength;

  unsigned long previousTime = 0;
  int currentPathSegment = 0;
  double currentDistanceAlongSegment = 0.0d;

public:
  FollowPath(KVector2* cp, int cpc, unsigned long executionTime);

  bool getPosition(unsigned long atDeltaTime, KVector2* targetPosition);

  ~FollowPath();

};

#endif
