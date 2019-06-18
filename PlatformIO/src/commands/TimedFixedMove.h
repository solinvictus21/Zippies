
#ifndef _TIMEDFIXEDMOVE_H_
#define _TIMEDFIXEDMOVE_H_

#include "ZippyMove.h"

class TimedFixedMove : public ZippyMove
{

private:
  unsigned long startTime;

protected:
  unsigned long executionTime;

  TimedFixedMove(unsigned long et)
    : executionTime(et)
  {}

  virtual void startTimed(ZippyController* zippy) = 0;
  virtual void loopTimed(double normalizedTime, ZippyController* zippy) = 0;

public:
  void start(unsigned long st, ZippyController* zippy)
  {
    startTime = st;
    startTimed(zippy);
    // SerialUSB.print("Started fixed time move at ");
    // SerialUSB.println(startTime);
  }

  unsigned long loop(unsigned long currentTime, ZippyController* zippy)
  {
    // SerialUSB.print("Updating fixed time move at ");
    // SerialUSB.println(currentTime);
    unsigned long deltaTime = currentTime - startTime;
    if (deltaTime > executionTime) {
      loopTimed(1.0d, zippy);
      return deltaTime - executionTime;
    }

    double normalizedTime = ((double)deltaTime) / ((double)executionTime);
    loopTimed(normalizedTime, zippy);
    return 0;
  }

  virtual ~TimedFixedMove() {}

};

#endif
