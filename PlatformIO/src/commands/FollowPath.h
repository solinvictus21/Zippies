
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
  unsigned long lastUpdateTime = 0;

  Zippy* zippy;
  KVector2* pathPoints;
  int pathPointCount;

  //the index into the pathPoints of the point representing the end of the current segment
  int currentControlPoint = 0;

  //the vector representing the start of the current segment
  KVector2 currentSegmentStart;

  //the relative path from the start to the end of the current segment
  KBezier2 currentSegment;

  //the distance we've driven along the current segment
  unsigned long pathStartTime = 0;
  double currentDistanceAlongSegment = 0.0d;

  double linearSetPoint = 0.0d;
  double linearInput = 0.0d;
  double linearOutput = 0.0d;
  PID linearPID;

  double rotationalSetPoint = 0.0d;
  double rotationalInput = 0.0d;
  double rotationalOutput = 0.0d;
  PID rotationalPID;

  void updateInputs(unsigned long currentTime);
  void calculateNextPosition(KVector2* nextPosition);
  void getCurrentTargetPosition(KVector2* nextPosition);

public:
  FollowPath(Zippy* zippy, KVector2* pathPoints, int pathPointCount);
  void start(unsigned long currentTime);
  bool loop(unsigned long currentTime);

};

#endif
