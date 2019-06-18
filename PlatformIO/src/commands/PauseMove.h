
#ifndef _PAUSEMOVE_H_
#define _PAUSEMOVE_H_

#include "ZippyMove.h"

class PauseMove : public ZippyMove
{

private:
  unsigned long pauseTime;

  unsigned long startTime;

public:
  PauseMove(unsigned long pt)
    : pauseTime(pt)
  {}

  void start(unsigned long st, ZippyController* zippy)
  {
    startTime = st;
  }

  unsigned long loop(unsigned long currentTime, ZippyController* zippy)
  {
    zippy->stopMoving();
    unsigned long deltaTime = currentTime - startTime;
    if (deltaTime > pauseTime)
      return deltaTime - pauseTime;

    return 0;
  }

};

#endif
