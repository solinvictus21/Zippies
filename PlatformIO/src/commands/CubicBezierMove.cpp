
#include "CubicBezierMove.h"

//we limit the length of the control points to 100mm to prevent issue with radical changes in orientation across large distances
#define CUBIC_CONTROL_POINT_LIMIT_MM 100.0d

void CubicBezierMove::startTimed(ZippyController* zippy)
{
  end();
  zippy->startMoving();
  startingPosition = new KVector2(&zippy->getTargetPosition()->vector);
  controlPoint1 = new KVector2();
  controlPoint2 = new KVector2();

  //calculate the two control points from the starting and ending orientations
  //the length of each control point is half the distance between the anchor points
  double controlPointLength = sqrt(pow(endingPosition.vector.getX() - startingPosition->getX(), 2.0d) +
      pow(endingPosition.vector.getY() - startingPosition->getY(), 2.0d)) / 2.0d;
  controlPoint1->set(0.0d, controlPointLength);
  controlPoint1->rotate(startingPosition->getOrientation());
  controlPoint1->addVector(startingPosition);

  controlPoint2->set(0.0d, controlPointLength);
  controlPoint2->rotate(addAngles(endingPosition.orientation, M_PI));
  controlPoint2->addVector(&endingPosition.vector);
}

void CubicBezierMove::loopTimed(double t, ZippyController* zippy)
{
  const KVector2* endPoint = &endingPosition.vector;

  double u = 1.0d - t;
  double u2 = u * u;
  double u3 = u2 * u;
  double t2 = t * t;
  double t3 = t2 * t;
  double u2t3 = 3.0d * u2 * t;
  double ut23 = 3.0d * u * t2;
  double interpolatedX = (u3 * startingPosition->getX()) + (u2t3 * controlPoint1->getX()) + (ut23 * controlPoint2->getX()) + (t3 * endPoint->getX());
  double interpolatedY = (u3 * startingPosition->getY()) + (u2t3 * controlPoint1->getY()) + (ut23 * controlPoint2->getY()) + (t3 * endPoint->getY());

  //dP(t) / dt = (-3(1-t)^2 * P0) + (3(1-t)^2 * P1) - (6t(1-t) * P1) - (3t^2 * P2) + (6t(1-t) * P2) + (3t^2 * P3)
  //           = -3u2
  //now calculate the orientation at that point
  double slope0x = getControlPoint(t, startingPosition->getX(), controlPoint1->getX(), controlPoint2->getX());
  double slope0y = getControlPoint(t, startingPosition->getY(), controlPoint1->getY(), controlPoint2->getY());
  double slope1x = getControlPoint(t, controlPoint1->getX(), controlPoint2->getX(), endPoint->getX());
  double slope1y = getControlPoint(t, controlPoint1->getY(), controlPoint2->getY(), endPoint->getY());

  double o = atan2(slope1x - slope0x, slope1y - slope0y);
  if (inReverse)
    o = addAngles(o, M_PI);
  zippy->move(interpolatedX, interpolatedY, o);
}

double getControlPoint(double t, double a, double b, double c)
{
  return a + ((a + c - (2.0d * b)) * t * t) + ((b - a) * 2.0d * t);
}
