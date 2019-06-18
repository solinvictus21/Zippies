
#include "BiArcMove.h"

void BiArcMove::startTimed(ZippyController* zippy)
{
  zippy->startMoving();
  //the following code is derived from formulae and helpful explanations provided from the following site
  //  http://www.ryanjuckett.com/programming/biarc-interpolation/

  //p1 = starting position
  const KPosition* startingPosition = zippy->getTargetPosition();
  double p1x = startingPosition->vector.getX();
  double p1y = startingPosition->vector.getY();

  //t1 = starting orientation
  //   = (sin(p1.o), cos(p1.o))
  double t1x = sin(startingPosition->orientation);
  double t1y = cos(startingPosition->orientation);

  //p2 = target position
  double p2x = endingPosition.vector.getX();
  double p2y = endingPosition.vector.getY();

  //t2 = target orientation
  //   = (sin(p2.o), cos(p2.o))
  double t2x = sin(endingPosition.orientation);
  double t2y = cos(endingPosition.orientation);

  //v = vector from starting position to ending position
  //  = p2 - p1
  //  = (p2.x - p1.x, p2.y - p1.y))
  double vx = p2x - p1x;
  double vy = p2y - p1y;

  double d;
  if (startingPosition->orientation == endingPosition.orientation) {
    /*
    if (y == 0.0d) {
      double pmx = p1x + (vx / 2.0d);
      double pmy = p1y + (vy / 2.0d);
      arc1 = new Arc(p1x, p1y, t1x, t1y, pmx, pmy);
      double knotO = addAngles(startingPosition->orientation, atan2(vx, vy));
      arc2 = new Arc(pmx, pmy, knotO, p2x, p2y);
      totalArcLength = arc1->getArcLength() + arc2->getArcLength();
      return;
    }
    */

    //in this situation, our denominator becomes zero and d goes to infinity; handle this with a different formula
    double vDotT2 = (vx * t1x) + (vy * t1y);
    d = ((vx * vx) + (vy * vy)) / (4.0d * vDotT2);
  }
  else {
    //v dot v = (v.x * v.x) + (v.y * v.y)
    double vDotV = pow(vx, 2.0d) + pow(vy, 2.0d);

    //t = t1 + t2
    //  = (sin(p1.o) + sin(p2.o), cos(p1.o) + cos(p2.o))
    double tx = t1x + t2x;
    double ty = t1y + t2y;
    //v dot t = (v.x * t.x)                               + (v.y * t.y)
    //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
    //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
    double vDotT = (vx * tx) + (vy * ty);

    //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
    //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
    double t1DotT2 = (t1x * t2x) + (t1y * t2y);

    //precalc = 2 * (1 - (t1 dot t2))
    double t1DotT2Inv2 = 2.0d * (1.0d - t1DotT2);
    d = ( -vDotT + sqrt( pow(vDotT, 2.0d) + ( t1DotT2Inv2 * vDotV ) ) )
        / t1DotT2Inv2;
  }

  //now find the "knot" (the connection point between the arcs)
  //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
  double pmx = ( p1x + p2x + (d * (t1x - t2x)) ) / 2.0d;
  double pmy = ( p1y + p2y + (d * (t1y - t2y)) ) / 2.0d;

  //calculate the the two arcs
  arc1 = new Arc(p1x, p1y, t1x, t1y, pmx, pmy);
  double knotO = addAngles(startingPosition->orientation, arc1->getDeltaAngle());
  arc2 = new Arc(pmx, pmy, knotO, p2x, p2y);
  totalArcLength = arc1->getArcLength() + arc2->getArcLength();
}

void BiArcMove::loopTimed(double normalizedTime, ZippyController* zippy)
{
  double distance = normalizedTime * totalArcLength;
  double x, y, orientation;
  double arcLength1 = arc1->getArcLength();
  if (distance < arcLength1) {
    //moving through first arc
    arc1->interpolate(distance / arcLength1, &x, &y, &orientation);
  }
  else {
    //moving through second arc
    distance -= arcLength1;
    arc2->interpolate(distance / arc2->getArcLength(), &x, &y, &orientation);
  }

  zippy->move(x, y, orientation);
}
