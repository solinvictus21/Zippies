
#ifndef _PAUSEMOVE_H_
#define _PAUSEMOVE_H_

#include "ZippyMove.h"

class PauseMove : public ZippyMove
{

private:
  bool inReverse;
  unsigned long pauseTime;

public:
  PauseMove(unsigned long t)
    : PauseMove(t, false)
  {}

  PauseMove(bool r, unsigned long t)
    : inReverse(r),
      pauseTime(t)
  {}

  unsigned long start(Zippy* zippy, const KPosition* sp) {
    zippy->setReverse(inReverse);
    return pauseTime;
  }

  void update(Zippy* zippy, double atNormalizedTime) const {
  }

};

#endif
