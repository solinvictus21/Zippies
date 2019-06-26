
#ifndef _TURN_H_
#define _TURN_H_

#include "ZPath.h"

class Turn : public ZPath
{
private:
  double startO;
  double deltaO;

public:
  Turn(double sto, double dto)
    : startO(sto),
      deltaO(dto)
  {}

  double getLength() const { return deltaO; }

  void interpolate(double normalizedTime, KPosition* position) const
  {
    position->orientation = addAngles(startO, deltaO * normalizedTime);
  }

};

#endif
