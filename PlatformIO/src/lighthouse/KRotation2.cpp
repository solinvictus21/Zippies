
#include "KRotation2.h"

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
  if (angle <= -M_PI)
    angle += 2.0d * M_PI;
  else if (angle > M_PI)
    angle -= 2.0d * M_PI;
  return angle;
}
