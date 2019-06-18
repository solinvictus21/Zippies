
#ifndef _LINEARMOVE_H_
#define _LINEARMOVE_H_

#include "TimedFixedMove.h"
#include "../lighthouse/KVector2.h"

class LinearMove : public TimedFixedMove
{

private:
  KVector2 targetPosition;
  double targetOrientation;

  const KPosition* startingPosition = NULL;

protected:
  void startTimed(ZippyController* zippy)
  {
    end();
    zippy->startMoving();
    startingPosition = new KPosition(zippy->getTargetPosition());
    targetOrientation = atan2(targetPosition.getX() - startingPosition->vector.getX(),
        targetPosition.getY() - startingPosition->vector.getY());
  }

  void loopTimed(double normalizedTime, ZippyController* zippy)
  {
    double currentX = startingPosition->vector.getX() +
        ((targetPosition.getX() - startingPosition->vector.getX()) * normalizedTime);
    double currentY = startingPosition->vector.getY() +
        ((targetPosition.getY() - startingPosition->vector.getY()) * normalizedTime);
    zippy->move(currentX, currentY, targetOrientation);
  }

public:
  LinearMove(double tx, double ty, unsigned long et)
    : TimedFixedMove(et),
      targetPosition(tx, ty)
  {}

  LinearMove(double tx, double ty, bool r, unsigned long et)
    : TimedFixedMove(et),
      targetPosition(tx, ty)
  {}

  void end() {
    if (startingPosition != NULL) {
      delete startingPosition;
      startingPosition = NULL;
    }
  }

  ~LinearMove() { end(); }

};

#endif
