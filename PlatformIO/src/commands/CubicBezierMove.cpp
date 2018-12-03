
#include "CubicBezierMove.h"

//we limit the length of the control points to 100mm to prevent issue with radical changes in orientation across large distances
#define CUBIC_CONTROL_POINT_LIMIT_MM 100.0d

unsigned long CubicBezierMove::start(Zippy* zippy, const KPosition* sp)
{
  startingPosition = sp;

  if (controlPoint1 == NULL)
    controlPoint1 = new KVector2();
  if (controlPoint2 == NULL)
    controlPoint2 = new KVector2();

  //calculate the two control points from the starting and ending orientations
  //the length of each control point is half the distance between the anchor points
  const KVector2* startPoint = &startingPosition->vector;
  const KVector2* endPoint = &endingPosition.vector;
  /*
  double controlPointLength = sqrt(pow(endPoint->getX() - startPoint->getX(), 2.0d) + pow(endPoint->getY() - startPoint->getY(), 2.0d)) / 2.0d;
  controlPoint1->set(0.0d, controlPointLength);
  controlPoint1->rotate((inReverse != zippy->isInReverse()) ? addAngles(startingPosition->orientation, M_PI) : startingPosition->orientation);
  controlPoint1->addVector(startPoint);
  controlPoint2->set(0.0d, controlPointLength);
  controlPoint2->rotate(addAngles(endingPosition.orientation, M_PI));
  controlPoint2->addVector(endPoint);
  // */
  double width = endingPosition.vector.getX() - startingPosition->vector.getX();
  double height = endingPosition.vector.getY() - startingPosition->vector.getY();
  controlPoint1->set(0.0d, 1.0d);
  controlPoint1->rotate((inReverse != zippy->isInReverse()) ? addAngles(startingPosition->orientation, M_PI) : startingPosition->orientation);
  controlPoint1->setD(abs((controlPoint1->getX() * width) + (controlPoint1->getY() * height)) / 2.0d);
  controlPoint1->addVector(startPoint);
  controlPoint2->set(0.0d, 1.0d);
  controlPoint2->rotate(addAngles(endingPosition.orientation, M_PI));
  controlPoint2->setD(abs((controlPoint2->getX() * width) + (controlPoint2->getY() * height)) / 2.0d);
  controlPoint2->addVector(endPoint);

  zippy->setReverse(inReverse);

  return executionTime;
}

void CubicBezierMove::update(Zippy* zippy, double t) const
{
  double u = 1.0d - t;
  double u2 = u * u;
  double u3 = u2 * u;
  double ut = u * t;
  double t2 = t * t;
  double t3 = t2 * t;
  double u2t3 = 3.0d * u2 * t;
  double ut23 = 3.0d * u * t2;

  //now calculate the orientation at that point
  const KVector2* startPoint = &startingPosition->vector;
  const KVector2* endPoint = &endingPosition.vector;
  double slope0x = getControlPoint(t, startPoint->getX(), controlPoint1->getX(), controlPoint2->getX());
  double slope0y = getControlPoint(t, startPoint->getY(), controlPoint1->getY(), controlPoint2->getY());
  double slope1x = getControlPoint(t, controlPoint1->getX(), controlPoint2->getX(), endPoint->getX());
  double slope1y = getControlPoint(t, controlPoint1->getY(), controlPoint2->getY(), endPoint->getY());

  zippy->move((u3 * startPoint->getX()) + (u2t3 * controlPoint1->getX()) + (ut23 * controlPoint2->getX()) + (t3 * endPoint->getX()),
      (u3 * startPoint->getY()) + (u2t3 * controlPoint1->getY()) + (ut23 * controlPoint2->getY()) + (t3 * endPoint->getY()),
      atan2(slope1x - slope0x, slope1y - slope0y));
}

double getControlPoint(double t, double a, double b, double c)
{
  return a + ((a + c - (2.0d * b)) * t * t) + ((b - a) * 2.0d * t);
}
