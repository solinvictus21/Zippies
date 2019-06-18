
#ifndef _TIMEDVARIABLEMOVE_H_
#define _TIMEDVARIABLEMOVE_H_

#include "ZippyMove.h"

class TimedVariableMove : public ZippyMove
{

private:
  unsigned long startTime = 0;
  unsigned long executionTime = 0;

protected:
  TimedVariableMove() {}

  virtual unsigned long startTimed(unsigned long startTime, ZippyController* zippy) = 0;
  virtual void loopTimed(double normalizedTime, ZippyController* zippy) const = 0;

public:
  void start(unsigned long st, ZippyController* zippy)
  {
    startTime = st;
    executionTime = startTimed(st, zippy);
  }

  unsigned long loop(unsigned long currentTime, ZippyController* zippy)
  {
    unsigned long deltaTime = currentTime - startTime;
    if (deltaTime >= executionTime) {
      loopTimed(1.0d, zippy);
      return deltaTime - executionTime;
    }

    double normalizedTime = ((double)deltaTime) / ((double)executionTime);
    loopTimed(normalizedTime, zippy);
    return 0;
  }

  virtual ~TimedVariableMove() {}

};

#endif
