
#ifndef _ARCMOVE_H_
#define _ARCMOVE_H_

#include "TimedFixedMove.h"

class ArcMove : public TimedFixedMove
{

private:
  double radius;
  double angle;

  const KPosition* startingPosition = NULL;
  KPosition center;

protected:
  void startTimed(ZippyController* zippy)
  {
    end();
    startingPosition = new KPosition(zippy->getTargetPosition());
    center.orientation = addAngles(startingPosition->orientation, -M_PI_2);
    center.vector.set(
        startingPosition->vector.getX() - (radius * sin(center.orientation)),
        startingPosition->vector.getY() - (radius * cos(center.orientation)));
    zippy->startMoving();
  }

  void loopTimed(double normalizedTime, ZippyController* zippy)
  {
    double deltaAngle = angle * normalizedTime;
    double angleOnArc = addAngles(center.orientation, deltaAngle);
    double x = center.vector.getX() + (radius * sin(angleOnArc));
    double y = center.vector.getY() + (radius * cos(angleOnArc));
    zippy->move(x, y, addAngles(startingPosition->orientation, deltaAngle));
  }

public:
  ArcMove(double r, double a, unsigned long et)
    : TimedFixedMove(et),
      radius(r),
      angle(a)
  {}

  void end() {
    if (startingPosition != NULL) {
      delete startingPosition;
      startingPosition = NULL;
    }
  }

  ~ArcMove() { end(); }

};

#endif
