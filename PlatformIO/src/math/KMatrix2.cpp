
#include "zippies/math/KMatrix2.h"

void calculateRelativeBiArcKnot(KMatrix2* relativeTargetPosition)
{
    calculateRelativeBiArcKnot(relativeTargetPosition, relativeTargetPosition);
}

void calculateRelativeBiArcKnot(const KMatrix2* relativeTargetPosition, KMatrix2* knot)
{
  if (relativeTargetPosition->orientation.get() == 0.0d) {
    //when the orientation is zero, this would create an asymptote in our bi-arc calculation because cos(0) = 1
    if (knot != relativeTargetPosition)
      knot->position.set(&relativeTargetPosition->position);
    knot->position.multiply(0.5d);
    knot->orientation.set(2.0d * knot->position.atan());
    return;
  }

  //use these calculations to find the appropriate d value for the bi-arc
  //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
  //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
  //          = t2.y
  double t2x = relativeTargetPosition->orientation.sin();
  double t2y = relativeTargetPosition->orientation.cos();

  //v dot t = (v.x * t.x)                               + (v.y * t.y)
  //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
  //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
  double vDotT =
      (relativeTargetPosition->position.getX() * t2x) +
      (relativeTargetPosition->position.getY() * (1.0 + t2y));

  //v dot v = (v.x * v.x) + (v.y * v.y)
  //        = distanceToTarget ^ 2
  double vDotV = relativeTargetPosition->position.getD2();
  double t1DotT2Inv2 = 2.0 * (1.0 - t2y);
  double discrim = sqrt(sq(vDotT) + ( t1DotT2Inv2 * vDotV ) );
  double d;
  if (relativeTargetPosition->position.getY() > 0.0) {
      d = (-vDotT + discrim) / t1DotT2Inv2;
  }
  else {
      d = (-vDotT - discrim) / t1DotT2Inv2;
  }

  //now we can use the d value to calculation the position of the "knot", which is the intersection
  //of the two arcs which make up the bi-arc
  knot->position.set(
    ( relativeTargetPosition->position.getX() + (d * (-t2x)) ) / 2.0,
    ( relativeTargetPosition->position.getY() + (d * (1.0 - t2y)) ) / 2.0);
  knot->orientation.set(2.0d * knot->position.atan());
}
