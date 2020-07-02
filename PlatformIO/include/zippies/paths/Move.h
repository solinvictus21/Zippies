
#ifndef _MOVE_H_
#define _MOVE_H_

#include "PathSegment.h"

class Move : public PathSegment
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

  Move(const ZMatrix2* start, double d)
    : Move(start->position.getX(), start->position.getY(), start->orientation.get(), d)
  {}

  // bool updatesPosition() const { return true; }
  // double getLength() const { return abs(distance); }

  void interpolate(
    double normalizedTime,
    ZMatrix2* position) const
  {
    double currentDistance = distance * normalizedTime;
    position->position.set(
      startX + (currentDistance * sin(startO)),
      startY + (currentDistance * cos(startO)));
    position->orientation.set(startO);
  }

};

#endif
