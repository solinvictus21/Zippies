
#ifndef _BIARCMOVE_H_
#define _BIARCMOVE_H_

#include "TimedFixedMove.h"
#include "../paths/Arc.h"
#include "../lighthouse/KPosition.h"

class BiArcMove : public TimedFixedMove
{

private:
  KPosition endingPosition;

  Arc* arc1 = NULL;
  Arc* arc2 = NULL;
  double totalArcLength;

protected:
  void startTimed(ZippyController* zippy);
  void loopTimed(double normalizedTime, ZippyController* zippy);

public:
  BiArcMove(double endx, double endy, double o, unsigned long et)
    : BiArcMove(endx, endy, o, false, et)
  {}

  BiArcMove(double endx, double endy, double o, bool r, unsigned long et)
    : TimedFixedMove(et)
  {
    endingPosition.vector.set(endx, endy);
    endingPosition.orientation = o;
  }

  void end() {
    if (arc1 != NULL) {
      delete arc1;
      arc1 = NULL;
    }
    if (arc2 != NULL) {
      delete arc2;
      arc2 = NULL;
    }
  }

  ~BiArcMove() { end(); }

};

#endif
