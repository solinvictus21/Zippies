
#ifndef _LINEARTURN_H_
#define _LINEARTURN_H_

#include "TimedFixedMove.h"

class LinearTurn : public TimedFixedMove
{

private:
  double targetOrientation;
  bool interpolated;

  double startingOrientation;

protected:
  void startTimed(ZippyController* zippy)
  {
    zippy->startMoving();
    startingOrientation = zippy->getTargetPosition()->orientation;
  }

  void loopTimed(double normalizedTime, ZippyController* zippy)
  {
    if (interpolated) {
      zippy->turn(addAngles(startingOrientation,
          (subtractAngles(targetOrientation, startingOrientation)) * normalizedTime));
    }
    else
      zippy->turn(targetOrientation);
  }

public:
  LinearTurn(double o, unsigned long et)
    : LinearTurn(o, et, true, false)
  {}

  LinearTurn(double o, unsigned long et, bool i)
    : LinearTurn(o, et, i, false)
  {}

  LinearTurn(double o, unsigned long et, bool i, bool r)
    : TimedFixedMove(et),
      targetOrientation(o),
      interpolated(i)
  {}

};

#endif
