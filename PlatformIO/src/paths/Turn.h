
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

  void interpolate(
    double normalizedTime,
    KMatrix2* position) const
  {
    position->orientation.set(addAngles(startO, deltaO * normalizedTime));
  }

};

#endif
