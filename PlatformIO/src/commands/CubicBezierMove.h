
#ifndef _CUBICBEZIERMOVE_H_
#define _CUBICBEZIERMOVE_H_

#include "TimedFixedMove.h"
#include "../lighthouse/KVector2.h"

class CubicBezierMove : public TimedFixedMove
{

private:
  KPosition endingPosition;
  bool inReverse;

  KVector2* startingPosition = NULL;
  KVector2* controlPoint1 = NULL;
  KVector2* controlPoint2 = NULL;

protected:
  void startTimed(ZippyController* zippy);
  void loopTimed(double normalizedTime, ZippyController* zippy);

public:
  CubicBezierMove(double endx, double endy, double o, unsigned long et)
    : CubicBezierMove(endx, endy, o, false, et)
  {}

  CubicBezierMove(double endx, double endy, double o, bool r, unsigned long et)
    : TimedFixedMove(et),
      endingPosition(endx, endy, o),
      inReverse(r)
  {
  }

  void end() {
    // TimedFixedMove::end();
    if (startingPosition != NULL) {
      delete startingPosition;
      startingPosition = NULL;
    }
    if (controlPoint1 != NULL) {
      delete controlPoint1;
      controlPoint1 = NULL;
    }
    if (controlPoint2 != NULL) {
      delete controlPoint2;
      controlPoint2 = NULL;
    }
  }

  ~CubicBezierMove() {
    end();
  }

};

double getControlPoint(double t, double a, double b, double c);

#endif
