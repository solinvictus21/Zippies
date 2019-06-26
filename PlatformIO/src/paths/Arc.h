
#ifndef _ARC_H_
#define _ARC_H_

#include "../lighthouse/KVector2.h"
#include "ZPath.h"

class Arc : public ZPath
{
private:
  KVector2 center;

  double startOrientation;
  double radius;
  double startAngle;
  double deltaAngle;
  double arcLength;

  void calculateArc(
    double startX, double startY, double startO,
    double endX, double endY)
  {
    this->startOrientation = startO;
    double deltaX = endX - startX;
    double deltaY = endY - startY;
    double deltaDotDelta = (deltaX * deltaX) + (deltaY * deltaY);
    double tangentX = sin(startO);
    double tangentY = cos(startO);
    double n2DotDelta = ((2.0 * -tangentY) * deltaX) + ((2.0 * tangentX) * deltaY);

    //the radius; a negative value indicates a turn forward to the right or backward to the left
    double s = deltaDotDelta / n2DotDelta;

    //c = the center point
    center.set(startX + (s * -tangentY), startY + (s * tangentX));
    this->startAngle = atan2(startX - center.getX(), startY - center.getY());
    double endAngle = atan2(endX - center.getX(), endY - center.getY());
    this->deltaAngle = subtractAngles(endAngle, startAngle);
    this->radius = abs(s);
    this->arcLength = abs(radius * deltaAngle);
  }

public:
  Arc(double startX, double startY, double startO,
      double endX, double endY)
    : startOrientation(startO)
  {
    calculateArc(startX, startY, startO, endX, endY);
  }

  double getDeltaAngle() const { return deltaAngle; }

  double getLength() const { return arcLength;}

  void interpolate(double normalizedTime, KPosition* position) const
  {
    double currentAngle = deltaAngle * normalizedTime;
    double angleOnArc = addAngles(startAngle, currentAngle);
    position->vector.set(center.getX() + (radius * sin(angleOnArc)),
      center.getY() + (radius * cos(angleOnArc)));
    position->orientation = addAngles(startOrientation, currentAngle);
  }

};

#endif
