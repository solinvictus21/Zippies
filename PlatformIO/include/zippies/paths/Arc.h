
#ifndef _ARC_H_
#define _ARC_H_

#include "PathSegment.h"
#include "zippies/ZippyMath.h"

class Arc : public PathSegment
{

private:
  double startOrientation;
  double deltaAngle;
  double radius;

  ZMatrix2 center;

public:
  Arc(
    const ZMatrix2* startPosition,
    const ZMatrix2* relativeTargetPosition)
  {
    this->startOrientation = startPosition->orientation.get();
    this->radius = relativeTargetPosition->position.getD() / (2.0 * sin(relativeTargetPosition->position.atan2()));
    this->deltaAngle = 2.0d * relativeTargetPosition->position.atan();
    this->center.set(radius, 0.0, -M_PI_2);
    this->center.concat(startPosition);
  }

  Arc(
    const ZMatrix2* startPosition,
    double radius, double subtendedAngle)
  {
    this->startOrientation = startPosition->orientation.get();
    this->radius = abs(radius);
    this->deltaAngle = subtendedAngle;
    this->center.set(radius, 0.0, radius < 0.0d ? M_PI_2 : -M_PI_2);
    this->center.concat(startPosition);
  }

  void interpolate(
    double normalizedTime,
    ZMatrix2* position) const
  {
    //determine the angle at the requested time
    double currentAngle = deltaAngle * normalizedTime;

    //calculate the target position
    double angleOnArc = addAngles(center.orientation.get(), currentAngle);
    position->position.set(
        center.position.getX() + (radius * sin(angleOnArc)),
        center.position.getY() + (radius * cos(angleOnArc)));

    //calculate the orientation at the target position
    position->orientation.set(addAngles(startOrientation, currentAngle));
  }

};

#endif
