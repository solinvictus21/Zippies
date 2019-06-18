
#ifndef _LINEARVELOCITYMOVE_H_
#define _LINEARVELOCITYMOVE_H_

#include "TimedVariableMove.h"
#include "../lighthouse/KPosition.h"

class LinearVelocityMove : public TimedVariableMove
{

private:
  KVector2 targetPosition;
  double targetVelocity;

  const KPosition* startingPosition = NULL;
  double targetOrientation;

  unsigned long startTimed(unsigned long st, ZippyController* zippy)
  {
    end();
    zippy->startMoving();
    startingPosition = new KPosition(zippy->getTargetPosition());
    double xOffset = targetPosition.getX() - startingPosition->vector.getX();
    double yOffset = targetPosition.getY() - startingPosition->vector.getY();
    targetOrientation = atan2(xOffset, yOffset);
    double distance = sqrt(pow(xOffset, 2.0d) + pow(yOffset, 2.0d));
    return (distance / targetVelocity) * 1000.0d;
  }

  void loopTimed(double normalizedTime, ZippyController* zippy) const
  {
    double currentX = startingPosition->vector.getX() +
        ((targetPosition.getX() - startingPosition->vector.getX()) * normalizedTime);
    double currentY = startingPosition->vector.getY() +
        ((targetPosition.getY() - startingPosition->vector.getY()) * normalizedTime);
    zippy->move(currentX, currentY, targetOrientation);
  }

public:
  LinearVelocityMove(double tx, double ty, double tv)
    : targetPosition(tx, ty),
      targetVelocity(tv)
  {}

  LinearVelocityMove(double tx, double ty, bool r, double tv)
    : targetPosition(tx, ty),
      targetVelocity(tv)
  {}

  void end() {
    if (startingPosition != NULL) {
      delete startingPosition;
      startingPosition = NULL;
    }
  }

  ~LinearVelocityMove() { end(); }

};

#endif
