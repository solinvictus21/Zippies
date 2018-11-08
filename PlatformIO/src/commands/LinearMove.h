
#ifndef _LINEARMOVE_H_
#define _LINEARMOVE_H_

#include "ZippyMove.h"
#include "../lighthouse/KVector2.h"

class LinearMove : public ZippyMove
{

private:
  const KPosition* startingPosition;
  KVector2 targetPosition;
  double targetOrientation;
  unsigned long executionTime;
  bool inReverse;

public:
  LinearMove(double tx, double ty, unsigned long t)
    : LinearMove(tx, ty, t, false)
  {}

  LinearMove(double tx, double ty, unsigned long t, bool r)
    : targetPosition(tx, ty),
      executionTime(t),
      inReverse(r)
  {}

  unsigned long start(Zippy* zippy, const KPosition* sp) {
    zippy->setReverse(inReverse);
    startingPosition = sp;
    targetOrientation = atan2(targetPosition.getX() - sp->vector.getX(),
        targetPosition.getY() - sp->vector.getY());
    return executionTime;
  }

  void update(Zippy* zippy, double atNormalizedTime) const {
    zippy->move(startingPosition->vector.getX() +
        ((targetPosition.getX() - startingPosition->vector.getX()) * atNormalizedTime),
        startingPosition->vector.getY() +
            ((targetPosition.getY() - startingPosition->vector.getY()) * atNormalizedTime),
        targetOrientation);
  }

};

#endif
