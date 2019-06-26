
#include "ZPathPlanner.h"
#include "Turn.h"
#include "Move.h"
#include "Arc.h"
#include "BiArc.h"

//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_EPSILON                   5.0d
#define LINEAR2_EPSILON                 25.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (1 degree)
#define ANGULAR_EPSILON                  0.017453292519943d

ZPath* planPath(const KPosition* start, const KPosition* end)
{
  return planPath(
    start->vector.getX(), start->vector.getY(), start->orientation,
    end->vector.getX(), end->vector.getY(), end->orientation);
}

ZPath* planPath(
  double startX, double startY, double startO,
  double endX, double endY, double endO)
{
  //the following code is derived from formulae and helpful explanations provided from the following site
  //  http://www.ryanjuckett.com/programming/biarc-interpolation

  // SerialUSB.println("Planning path");
  double deltaX = endX - startX;
  double deltaY = endY - startY;
  KPosition relativeTargetPosition = new KPosition(deltaX, deltaY, subtractAngles(endO, startO));
  relativeTargetPosition.vector.rotate(-startO);

  if (distance2Zero(relativeTargetPosition.vector.getD2())) {
      //no movement
      if (angleZero(relativeTargetPosition.orientation)) {
          //no turn do nothing
          return NULL;
      }

      //linear turn
      return new Turn(startO, subtractAngles(endO, startO));
  }

  // SerialUSB.println("Distance is non-zero");
  if (angleZero(relativeTargetPosition.orientation)) {
      //orientation at target is equivalent to the starting orientation
      if (distanceZero(relativeTargetPosition.vector.getX())) {
          //target is directly in front or behind
          //linear move
          // SerialUSB.println("Planned linear move.");
          return new Move(startX, startY, startO, relativeTargetPosition.vector.getY());
      }

      //now find the "knot" (the connection point between the arcs)
      //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
      double pmx = ( startX + endX ) / 2.0;
      double pmy = ( startY + endY ) / 2.0;
      Arc* arc1 = new Arc(startX, startY, startO, pmx, pmy);
      Arc* arc2 = new Arc(pmx, pmy, addAngles(startO, arc1->getDeltaAngle()), endX, endY);
      // SerialUSB.println("Planned complex bi-arc to matching orientation");
      return new BiArc(arc1, arc2);
  }


  double orientationAtTarget = addAngles(
      relativeTargetPosition.vector.getOrientation(),
      relativeTargetPosition.vector.getOrientation());
  if (anglesEquivalent(orientationAtTarget, relativeTargetPosition.orientation)) {
      // SerialUSB.println("Planned simple arc.");
      return new Arc(startX, startY, startO, endX, endY);
  }

  double t1x = sin(startO);
  double t1y = cos(startO);
  double t2x = sin(endO);
  double t2y = cos(endO);

  //v dot v = (v.x * v.x) + (v.y * v.y)
  double vDotV = pow(deltaX, 2.0) + pow(deltaY, 2.0);

  //t = t1 + t2
  //  = (sin(p1.o) + sin(p2.o), cos(p1.o) + cos(p2.o))
  //v dot t = (v.x * t.x)                               + (v.y * t.y)
  //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
  //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
  double vDotT = (deltaX * (t1x + t2x)) + (deltaY * (t1y + t2y));
  // print("vDotT is zero:", distanceZero(vDotT))

  //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
  //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
  double t1DotT2 = (t1x * t2x) + (t1y * t2y);

  // print("Planned complex bi-arc.")
  //precalc = 2 * (1 - (t1 dot t2))
  double t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2);
  double discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) );

  double d;
  double turnTowardTarget = 2.0d * relativeTargetPosition.vector.getOrientation();
  double turnTowardOrientation = subtractAngles(relativeTargetPosition.orientation, turnTowardTarget);
  if (abs(turnTowardTarget + turnTowardOrientation) <= M_PI) {
    //move forward
    d = ( -vDotT + discrim ) / t1DotT2Inv2;
  }
  else {
    //move backward
    d = ( -vDotT - discrim ) / t1DotT2Inv2;
  }

  //now find the "knot" (the connection point between the arcs)
  //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
  double pmx = ( startX + endX + (d * (t1x - t2x)) ) / 2.0;
  double pmy = ( startY + endY + (d * (t1y - t2y)) ) / 2.0;

  //calculate the the two arcs
  if (distance2Zero(pow(pmx - startX, 2.0) + pow(pmy - startY, 2.0)))
      return new Arc(pmx, pmy, startO, endX, endY);
  if (distance2Zero(pow(pmx - endX, 2.0) + pow(pmy - endY, 2.0)))
      return new Arc(startX, startY, startO, pmx, pmy);

  Arc* arc1 = new Arc(startX, startY, startO, pmx, pmy);
  // SerialUSB.println("Planned complex bi-arc move forward");
  return new BiArc(
    arc1,
    new Arc(pmx, pmy, addAngles(startO, arc1->getDeltaAngle()), endX, endY));
}

bool distanceZero(double distance)
{
  return abs(distance) <= LINEAR_EPSILON;
}

bool distance2Zero(double distance2)
{
  return abs(distance2) <= LINEAR2_EPSILON;
}

bool positionsEquivalent(const KPosition* p1, const KPosition* p2)
{
  return distance2Zero(pow(p2->vector.getX() - p1->vector.getX(), 2.0d) +
      pow(p2->vector.getY() - p1->vector.getY(), 2.0d));
}

bool angleZero(double angle)
{
  return abs(angle) <= ANGULAR_EPSILON;
}

bool anglesEquivalent(double angle1, double angle2)
{
  return abs(subtractAngles(angle2, angle1)) <= ANGULAR_EPSILON;
}
