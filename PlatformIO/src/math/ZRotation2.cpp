
#include "zippies/math/ZRotation2.h"

double subtractAngles(double a1, double a2)
{
  return snapAngle(a1 - a2);
}

double addAngles(double a1, double a2)
{
  return snapAngle(a1 + a2);
}

double snapAngle(double angle)
{
  while (angle <= -M_PI)
    angle += 2.0 * M_PI;
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  return angle;
}
