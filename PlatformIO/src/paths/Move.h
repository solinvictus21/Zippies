
#ifndef _MOVE_H_
#define _MOVE_H_

#include "ZPath.h"

class Move : public ZPath
{
private:
  double startX, startY, startO;
  double distance;

public:
  Move(double sx, double sy, double o, double d)
    : startX(sx),
      startY(sy),
      startO(o),
      distance(d)
  {}

  double getLength() const { return distance; }

  void interpolate(
    double normalizedTime,
    KPosition* position,
    bool* reverseMotion) const
  {
    double currentDistance = distance * normalizedTime;
    position->vector.set(
      startX + (currentDistance * sin(startO)),
      startY + (currentDistance * cos(startO)));
    position->orientation = startO;
    *reverseMotion = distance < 0.0d;
  }

};

#endif
