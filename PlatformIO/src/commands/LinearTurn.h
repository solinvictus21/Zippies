
#ifndef _LINEARTURN_H_
#define _LINEARTURN_H_

#include "ZippyMove.h"

class LinearTurn : public ZippyMove
{

private:
  const KPosition* startingPosition;
  double targetOrientation;
  unsigned long executionTime;
  bool interpolated;
  bool inReverse;

public:
  LinearTurn(double o, unsigned long t)
    : LinearTurn(o, t, true, false)
  {}

  LinearTurn(double o, unsigned long t, bool i)
    : LinearTurn(o, t, i, false)
  {}

  LinearTurn(double o, unsigned long t, bool i, bool r)
    : targetOrientation(o),
      executionTime(t),
      interpolated(i),
      inReverse(r)
  {}

  unsigned long start(Zippy* zippy, const KPosition* sp) {
    zippy->setReverse(inReverse);
    startingPosition = sp;
    return executionTime;
  }

  void update(Zippy* zippy, double atNormalizedTime) const {
    if (interpolated) {
      zippy->turn(addAngles(startingPosition->orientation,
          (subtractAngles(targetOrientation, startingPosition->orientation)) * atNormalizedTime));
    }
    else
      zippy->turn(targetOrientation);
  }

  void end() {
    startingPosition = NULL;
  }

};

#endif
