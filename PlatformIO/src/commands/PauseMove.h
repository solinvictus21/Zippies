
#ifndef _PAUSEMOVE_H_
#define _PAUSEMOVE_H_

#include "ZippyMove.h"

class PauseMove : public ZippyMove
{

private:
  unsigned long pauseTime;

public:
  PauseMove(unsigned long t)
    : pauseTime(t)
  {}

  unsigned long start(Zippy* zippy, const KPosition* sp) {
    return pauseTime;
  }

  void update(Zippy* zippy, double atNormalizedTime) const {
  }

};

#endif
