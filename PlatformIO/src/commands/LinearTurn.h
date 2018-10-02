
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

public:
  LinearTurn(double o, unsigned long t)
    : LinearTurn(o, t, true)
  {}

  LinearTurn(double o, unsigned long t, bool i)
    : targetOrientation(o),
      executionTime(t),
      interpolated(i)
  {}

  unsigned long start(Zippy* zippy, const KPosition* sp) {
    startingPosition = sp;
    if (!interpolated)
      zippy->turn(targetOrientation);
    return executionTime;
  }

  void update(Zippy* zippy, double atNormalizedTime) const {
    if (!interpolated)
      return;

    zippy->turn(addAngles(startingPosition->orientation,
        (subtractAngles(targetOrientation, startingPosition->orientation)) * atNormalizedTime));
  }

};



#endif
