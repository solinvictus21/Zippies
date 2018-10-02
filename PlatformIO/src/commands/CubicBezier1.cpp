
#include "CubicBezier1.h"

//we limit the length of the control points to 100mm to prevent issue with radical changes in orientation across large distances
#define CUBIC_CONTROL_POINT_LIMIT_MM 100.0d

double CubicBezier1::lerp(double t) const
{
  double u = 1.0d - t;
  double u2 = u * u;
  double u3 = u2 * u;
  double t2 = t * t;
  double t3 = t2 * t;

  double u2t3 = 3.0d * u2 * t;
  double ut23 = 3.0d * u * t2;
  return (u3 * a) + (u2t3 * b) + (ut23 * c) + (t3 * d);
}

double cubicLerp(double t,
                 double a1,
                 double c1,
                 double a2,
                 double c2)
{
  double a1c1 = a1+((c1-a1)*t);
  double c1c2 = c1+((c2-c1)*t);
  double c2a2 = c2+((a2-c2)*t);
  double a1c1c1c2 = a1c1+((c1c2-a1c1)*t);
  double c1c2c2a2 = c1c2+((c2a2-c1c2)*t);
  return a1c1c1c2+((c1c2c2a2-a1c1c1c2)*t);
}
