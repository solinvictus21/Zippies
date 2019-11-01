
#include "ZPathPlanner.h"
#include "Turn.h"
#include "Move.h"
#include "Arc.h"
#include "CompositePath.h"

const ZPath* planPath(
  const KMatrix2* start,
  double endX, double endY, double endO)
{
  KMatrix2 end(endX, endY, endO);
  return planPath(start, &end);
}

const ZPath* planPath(const KMatrix2* start, const KMatrix2* toPosition)
{
  //determine if we need to plan a bi-arc move first
  KMatrix2 relativeTarget(toPosition);
  relativeTarget.unconcat(start);
  if (!requiresBiArcMove(&relativeTarget))
    return planRelativePath(start, &relativeTarget);

  //plan a bi-arc path move
  KMatrix2 knot;
  calculateRelativeBiArcKnot(&relativeTarget, &knot);

  const ZPath* firstSegment = planRelativePath(start, &knot);

  relativeTarget.unconcat(&knot);
  knot.concat(start);
  const ZPath* secondSegment = planRelativePath(&knot, &relativeTarget);

  if (!firstSegment)
    return secondSegment;
  else if (!secondSegment)
    return firstSegment;

  return new CompositePath(firstSegment, secondSegment);
}

const ZPath* planRelativePath(const KMatrix2* start, const KMatrix2* relativeTarget)
{
  if (relativeTarget->position.getD2() < DOUBLE_EPSILON) {
    double deltaAngle = relativeTarget->orientation.get();
    if (abs(deltaAngle) < DOUBLE_EPSILON)
      return NULL;

    return new Turn(start->orientation.get(), deltaAngle);
  }

  if (abs(relativeTarget->position.atan()) < DOUBLE_EPSILON)
    return new Move(start, relativeTarget->position.getY());

  return new Arc(start, relativeTarget);
}

double calculateBiArcD(double vDotV,
                       double vDotT,
                       double t1DotT2);

void calculateRelativeBiArcKnot(KMatrix2* relativeTargetPosition)
{
  calculateRelativeBiArcKnot(relativeTargetPosition, relativeTargetPosition);
}

void calculateRelativeBiArcKnot(const KMatrix2* relativeTargetPosition,
                                KMatrix2* knotPosition)
{
  if (abs(relativeTargetPosition->orientation.get()) < DOUBLE_EPSILON) {
    knotPosition->position.set(
      relativeTargetPosition->position.getX() / 2.0d,
      relativeTargetPosition->position.getY() / 2.0d);
    knotPosition->orientation.set(2.0d * knotPosition->position.atan());
    return;
  }

  double t2x = sin(relativeTargetPosition->orientation.get());
  double t2y = cos(relativeTargetPosition->orientation.get());

  //v dot v = (v.x * v.x) + (v.y * v.y)
  //        = distanceToTarget ^ 2
  //v dot t = (v.x * t.x)                               + (v.y * t.y)
  //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
  //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
  //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
  //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
  //          = t2.y

  //use these calculations to find the appropriate d value for the bi-arc
  double vDotT = (relativeTargetPosition->position.getX() * t2x) +
      (relativeTargetPosition->position.getY() * (1.0 + t2y));
  double d = calculateBiArcD(
      relativeTargetPosition->position.getD2(),
      vDotT,
      t2y);

  //now we can use the d value to calculation the position of the "knot", which is the intersection
  //of the two arcs which make up the bi-arc
  knotPosition->position.set(
      ( relativeTargetPosition->position.getX() + (d * (-t2x)) ) / 2.0,
      ( relativeTargetPosition->position.getY() + (d * (1.0 - t2y)) ) / 2.0);
  knotPosition->orientation.set(2.0d * knotPosition->position.atan());
}

double calculateBiArcD(
  double vDotV,
  double vDotT,
  double t1DotT2)
{
    //precalc = 2 * (1 - (t1 dot t2))
    double t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2);
    double discrim = sqrt( sq(vDotT) + ( t1DotT2Inv2 * vDotV ) );

    //now find the smallest d value of the bi-arc to create the shortest bi-arc to the target
    double d = -vDotT + discrim;
    double altD = -vDotT - discrim;
    if (abs(d) > abs(altD)) {
        return altD / t1DotT2Inv2;
    }

    return d / t1DotT2Inv2;
}

bool requiresBiArcMove(const KMatrix2* relativeTarget)
{
  //a simple turn in place does not need a bi-arc
  if (relativeTarget->position.getD2() < DOUBLE_EPSILON)
    return false;

  //a simple linear move forward or backward does not need a bi-arc
  double directionToTarget = relativeTarget->position.atan();
  if (abs(directionToTarget) < DOUBLE_EPSILON)
    return false;

  //a simple arc move that arrives at the torget position in the correct orientation also
  //does not require a bi-arc move
  double subtendedAngle = 2.0d * directionToTarget;
  // if (abs(subtractAngles(subtendedAngle, relativeTarget->orientation.get())) < ANGULAR_POSITION_EPSILON)
  if (abs(subtractAngles(subtendedAngle, relativeTarget->orientation.get())) < DOUBLE_EPSILON)
    return false;

  return true;
}
