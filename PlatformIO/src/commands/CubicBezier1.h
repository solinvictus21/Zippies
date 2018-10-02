
#ifndef _CUBICBEZIER1_H_
#define _CUBICBEZIER1_H_

#include "../lighthouse/KVector2.h"
#include "../lighthouse/KPosition.h"

class CubicBezier1
{

private:
  double a;
  double b;
  double c;
  double d;

public:
  CubicBezier1(double a1, double b1, double c1, double d1)
    : a(a1),
      b(b1),
      c(c1),
      d(d1)
  {}

  double lerp(double atNormalizedTime) const;

};

double cubicLerp(double t,
                 double a1,
                 double c1,
                 double a2,
                 double c2);

#endif
