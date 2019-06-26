
#ifndef _ZPATHPLANNER_H_
#define _ZPATHPLANNER_H_

#include "ZPath.h"

ZPath* planPath(
  double startX, double startY, double startO,
  double endX, double endY, double endO);

extern bool distanceZero(double distance);
extern bool distance2Zero(double distance2);
extern bool positionsEquivalent(const KPosition* p1, const KPosition* p2);
extern bool angleZero(double angle);
extern bool anglesEquivalent(double angle1, double angle2);
extern bool isTurnTowardTarget(const KPosition* relativeTargetPosition);

#endif
