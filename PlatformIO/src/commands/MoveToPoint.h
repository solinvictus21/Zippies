
#ifndef _MOVETOPOINT_H_
#define _MOVETOPOINT_H_

#include "ZippyCommand.h"
#include "../lighthouse/KVector2.h"
#include "LinePath.h"

class MoveToPoint : public PathCommand
{

private:
  KVector2 startingPosition;
  KVector2 endingPosition;
  LinePath linePath;

public:
  MoveToPoint(double x, double y, unsigned long et)
    : PathCommand(&linePath, et),
      endingPosition(x, y),
      linePath(&startingPosition, &endingPosition)
  {}

  void setStartingPosition(double x, double y) {
    startingPosition.set(x, y);
  }

};

#endif
