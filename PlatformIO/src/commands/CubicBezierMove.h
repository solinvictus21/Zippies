
#ifndef _CUBICBEZIERMOVE_H_
#define _CUBICBEZIERMOVE_H_

#include "ZippyMove.h"
#include "../lighthouse/KVector2.h"

class CubicBezierMove : public ZippyMove
{

private:
  const KPosition* startingPosition;
  bool inReverse;
  KVector2* controlPoint1 = NULL;
  KVector2* controlPoint2 = NULL;
  KPosition endingPosition;
  unsigned long executionTime;

  void releaseControlPoints() {
    if (controlPoint1 != NULL) {
      delete controlPoint1;
      controlPoint1 = NULL;
    }
    if (controlPoint2 != NULL) {
      delete controlPoint2;
      controlPoint2 = NULL;
    }
  }

public:
  CubicBezierMove(double endx, double endy, double o, unsigned long t)
    : CubicBezierMove(endx, endy, o, false, t)
  {}

  CubicBezierMove(double endx, double endy, double o, bool r, unsigned long t)
    : inReverse(r),
      executionTime(t)
  {
    endingPosition.vector.set(endx, endy);
    endingPosition.orientation = o;
  }

  unsigned long start(Zippy* zippy, const KPosition* sp);

  void update(Zippy* zippy, double atNormalizedTime) const;

  void end() {
    ZippyMove::end();
    releaseControlPoints();
  }

  ~CubicBezierMove() {
    releaseControlPoints();
  }

};

double getControlPoint(double t, double a, double b, double c);

#endif
