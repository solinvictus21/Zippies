
#ifndef _ZPATHPLANNER_H_
#define _ZPATHPLANNER_H_

#include "../lighthouse/KMatrix2.h"
#include "ZPath.h"

const ZPath* planPath(
  const KMatrix2* start,
  double endX, double endY, double endO);
const ZPath* planPath(const KMatrix2* fromPosition, const KMatrix2* toPosition);
const ZPath* planRelativePath(const KMatrix2* start, const KMatrix2* relativeTarget);
void calculateRelativeBiArcKnot(KMatrix2* relativeTargetPosition);
bool requiresBiArcMove(const KMatrix2* relativeTarget);

#endif
