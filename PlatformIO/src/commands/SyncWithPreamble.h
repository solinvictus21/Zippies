
#ifndef _SYNCWITHPREAMBLE_H_
#define _SYNCWITHPREAMBLE_H_

#include "ZippyMove.h"

class SyncWithPreamble : public ZippyMove
{

public:
  SyncWithPreamble() {}

  void start(unsigned long st, ZippyController* zippy)
  {
    zippy->waitForPreamble();
  }

  unsigned long loop(unsigned long currentTime, ZippyController* zippy)
  {
    zippy->stopMoving();
    return zippy->foundPreamble();
  }

};

#endif
