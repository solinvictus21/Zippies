
#ifndef _ARC_H_
#define _ARC_H_

#include "../lighthouse/KVector2.h"
#include "ZPath.h"

class Arc : ZPath
{
private:
  KVector2 center;

  double startOrientation;
  double radius;
  double startAngle;
  double deltaAngle;
  double arcLength;

  void calculateArc(double startX, double startY, double tangentX, double tangentY,
      double endX, double endY)
  {
    //pmp = the vector from the starting point to the ending point
    double deltaX = endX - startX;
    double deltaY = endY - startY;
    double deltaDotDelta = (deltaX * deltaX) + (deltaY * deltaY);
    double n2DotDelta = ((2.0 * -tangentY) * deltaX) + ((2.0 * tangentX) * deltaY);

    //the radius; a negative value indicates a turn forward to the right or backward to the left
    radius = deltaDotDelta / n2DotDelta;

    //c = the center point
    center.set(startX + (radius * -tangentY), startY + (radius * tangentX));
    startAngle = atan2(startX - center.getX(), startY - center.getY());
    deltaAngle = subtractAngles(atan2(endX - center.getX(), endY - center.getY()), startAngle);
    radius = abs(radius);
    arcLength = abs(radius * deltaAngle);
  }

public:
  Arc(double startX, double startY, double o,
      double endX, double endY)
    : startOrientation(o)
  {
    calculateArc(startX, startY, sin(startOrientation), cos(startOrientation), endX, endY);
  }

  Arc(double startX, double startY, double tangentX, double tangentY,
      double endX, double endY)
    : startOrientation(atan2(tangentX, tangentY))
  {
    calculateArc(startX, startY, tangentX, tangentY, endX, endY);
  }

  double getDeltaAngle() { return deltaAngle; }

  double getArcLength() { return arcLength;}

  void interpolate(double normalizedTime, double* x, double* y, double* orientation)
  {
      double currentAngle = deltaAngle * normalizedTime;
      double angleOnArc = addAngles(startAngle, currentAngle);
      *x = center.getX() + (radius * sin(angleOnArc));
      *y = center.getY() + (radius * cos(angleOnArc));
      *orientation = addAngles(startOrientation, currentAngle);
  }

  void interpolate(double normalizedTime, KPosition* position)
  {
    double x, y, o;
    interpolate(normalizedTime, &x, &y, &o);
    position->vector.set(x, y);
    position->orientation = o;
  }

};

#endif
